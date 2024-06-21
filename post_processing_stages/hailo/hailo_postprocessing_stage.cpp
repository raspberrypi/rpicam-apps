/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_postprocessing_stage.cpp - Hailo software stage base class and helpers
 */

#include <algorithm>
#include <string>
#include <sys/mman.h>

#include "hailo_postprocessing_stage.hpp"

#include "hailo_postproc_lib.h"

using namespace hailort;

Allocator::Allocator()
{
}

Allocator::~Allocator()
{
	Reset();
}

void Allocator::Reset()
{
	std::scoped_lock<std::mutex> l(lock_);

	for (auto &info : alloc_info_)
		munmap(info.ptr, info.size);

	alloc_info_.clear();
}

std::shared_ptr<uint8_t> Allocator::Allocate(unsigned int size)
{
	std::scoped_lock<std::mutex> l(lock_);
	uint8_t *ptr = nullptr;

	auto info = std::find_if(alloc_info_.begin(), alloc_info_.end(),
							 [size](const AllocInfo &info) { return info.free && info.size == size; });
	if (info != alloc_info_.end())
	{
		info->free = false;
		ptr = info->ptr;
	}

	if (!ptr)
	{
		void *addr = mmap(NULL, size, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
		if (addr == MAP_FAILED)
			return {};

		ptr = static_cast<uint8_t *>(addr);
		alloc_info_.emplace_back(ptr, size, false);
	}

	return std::shared_ptr<uint8_t>(ptr, [this](uint8_t *ptr) { this->free(ptr); });
}

void Allocator::free(uint8_t *ptr)
{
	std::scoped_lock<std::mutex> l(lock_);

	auto info =	std::find_if(alloc_info_.begin(), alloc_info_.end(),
							 [ptr](const AllocInfo &info) { return info.ptr == ptr; });
	if (info != alloc_info_.end())
		info->free = true;
}

HailoPostProcessingStage::HailoPostProcessingStage(RPiCamApp *app)
	: PostProcessingStage(app)
{
}

HailoPostProcessingStage::~HailoPostProcessingStage()
{
	if (init_)
		configured_infer_model_->shutdown();
}

void HailoPostProcessingStage::Read(boost::property_tree::ptree const &params)
{
	hef_file_ = params.get<std::string>("hef_file");
}

void HailoPostProcessingStage::Configure()
{
	output_stream_ = app_->GetMainStream();
	raw_stream_ = app_->RawStream();
	low_res_stream_ = app_->LoresStream();
	if (low_res_stream_)
		low_res_info_ = app_->GetStreamInfo(low_res_stream_);
	if (output_stream_)
		output_stream_info_ = app_->GetStreamInfo(output_stream_);

	if (!init_)
	{
		if (!configureHailoRT())
			init_ = true;
	}

	allocator_.Reset();
	last_frame_ = {};
}

int HailoPostProcessingStage::configureHailoRT()
{
	Expected<std::unique_ptr<VDevice>> vdevice_exp = VDevice::create();
	if (!vdevice_exp)
	{
		LOG_ERROR("Failed create vdevice, status = " << vdevice_exp.status());
		return vdevice_exp.status();
	}
	vdevice_ = vdevice_exp.release();

	// Create infer model from HEF file.
	Expected<std::shared_ptr<InferModel>> infer_model_exp = vdevice_->create_infer_model(hef_file_);
	if (!infer_model_exp)
	{
		LOG_ERROR("Failed to create infer model, status = " << infer_model_exp.status());
		return infer_model_exp.status();
	}
	infer_model_ = infer_model_exp.release();
	infer_model_->set_hw_latency_measurement_flags(HAILO_LATENCY_MEASURE);

	// Configure the infer model
	//infer_model_->output()->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);
	Expected<ConfiguredInferModel> configured_infer_model_exp = infer_model_->configure();
	if (!configured_infer_model_exp)
	{
		LOG_ERROR("Failed to create configured infer model, status = " << configured_infer_model_exp.status());
		return configured_infer_model_exp.status();
	}
	configured_infer_model_ = std::make_shared<ConfiguredInferModel>(configured_infer_model_exp.release());

	// Create infer bindings
	Expected<ConfiguredInferModel::Bindings> bindings_exp = configured_infer_model_->create_bindings();
	if (!bindings_exp)
	{
		LOG_ERROR("Failed to create infer bindings, status = " << bindings_exp.status());
		return bindings_exp.status();
	}
	bindings_ = std::move(bindings_exp.release());

	hailo_3d_image_shape_t shape = infer_model_->inputs()[0].shape();
	input_tensor_size_ = libcamera::Size(shape.width, shape.height);

	return 0;
}

hailo_status HailoPostProcessingStage::DispatchJob(const uint8_t *input, AsyncInferJob &job,
												   std::vector<OutTensor> &output_tensors)
{
	hailo_status status;

	std::scoped_lock<std::mutex> l(lock_);

	// Input tensor.
	const std::string &input_name = infer_model_->get_input_names()[0];
	size_t input_frame_size = infer_model_->input(input_name)->get_frame_size();

	status = bindings_.input(input_name)->set_buffer(MemoryView((void *)(input), input_frame_size));
	if (status != HAILO_SUCCESS)
	{
		LOG_ERROR("Could not write to input stream with status " << status);
		return status;
	}

	// Output tensors.
	for (auto const &output_name : infer_model_->get_output_names())
	{
		size_t output_size = infer_model_->output(output_name)->get_frame_size();
		std::shared_ptr<uint8_t> output_buffer = allocator_.Allocate(output_size);
		if (!output_buffer)
		{
			LOG_ERROR("Could not allocate an output buffer!");
			return status;
		}

		status = bindings_.output(output_name)->set_buffer(MemoryView(output_buffer.get(), output_size));
		if (status != HAILO_SUCCESS)
		{
			LOG_ERROR("Failed to set infer output buffer, status = " << status);
			return status;
		}

		const std::vector<hailo_quant_info_t> quant = infer_model_->output(output_name)->get_quant_infos();
		const hailo_3d_image_shape_t shape = infer_model_->output(output_name)->shape();
		const hailo_format_t format = infer_model_->output(output_name)->format();
		output_tensors.emplace_back(std::move(output_buffer), output_name, quant[0], shape, format);
	}

	// Waiting for available requests in the pipeline.
	status = configured_infer_model_->wait_for_async_ready(1s);
	if (status != HAILO_SUCCESS)
	{
		LOG_ERROR("Failed to wait for async ready, status = " << status);
		return status;
	}

	Expected<LatencyMeasurementResult> inf_time_exp = configured_infer_model_->get_hw_latency_measurement();
	std::chrono::time_point<std::chrono::steady_clock> this_frame = std::chrono::steady_clock::now();

	if (inf_time_exp && last_frame_.time_since_epoch() != 0s)
	{
		const auto inf_time =
			std::chrono::duration_cast<std::chrono::milliseconds>(inf_time_exp.release().avg_hw_latency);
		const auto frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(this_frame - last_frame_);

		if (frame_time < inf_time)
			LOG(2, "Warning: model inferencing time of " << inf_time.count() << "ms " <<
				   "> current job interval of " << frame_time.count() << "ms!");
	}

	last_frame_ = this_frame;

	// Dispatch the job.
	Expected<AsyncInferJob> job_exp = configured_infer_model_->run_async(bindings_);
	if (!job_exp)
	{
		LOG_ERROR("Failed to start async infer job, status = " << job_exp.status());
		return status;
	}
	job = job_exp.release();

	// Detach and let the job run.
	job.detach();

	return status;
}

HailoROIPtr HailoPostProcessingStage::MakeROI(const std::vector<OutTensor> &output_tensors) const
{
	HailoROIPtr roi = std::make_shared<HailoROI>(HailoROI(HailoBBox(0.0f, 0.0f, 1.0f, 1.0f)));

	for (auto const &t : output_tensors)
	{
		hailo_vstream_info_t info;

		strncpy(info.name, t.name.c_str(), sizeof(info.name));
		// To keep GCC quiet...
		info.name[HAILO_MAX_STREAM_NAME_SIZE - 1] = '\0';
		info.format = t.format;
		info.quant_info = t.quant_info;
		if (HailoRTCommon::is_nms(info))
			info.nms_shape = infer_model_->outputs()[0].get_nms_shape().release();
		else
			info.shape = t.shape;

		roi->add_tensor(std::make_shared<HailoTensor>(t.data.get(), info));
	}

	return roi;
}
