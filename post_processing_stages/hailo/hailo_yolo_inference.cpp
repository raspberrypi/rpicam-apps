/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_yolo_inference.cpp - Hailo inference for yolo5/7/8 models
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <libcamera/geometry.h>

#include "core/rpicam_app.hpp"
#include "post_processing_stages/object_detect.hpp"

#include "detection/yolo_postprocess.hpp"

#include "hailo_postprocessing_stage.hpp"

using Size = libcamera::Size;
using PostProcFuncPtr = void (*)(HailoROIPtr, YoloParams *);
using PostProcFuncPtrNms = void (*)(HailoROIPtr);
using InitFuncPtr = YoloParams *(*)(std::string, std::string);
using FreeFuncPtr = void (*)(void *);

namespace fs = std::filesystem;

#define NAME "hailo_yolo_inference"
#define POSTPROC_LIB "libyolo_post.so"
#define POSTPROC_LIB_NMS "libyolo_hailortpp_post.so"

class YoloInference : public HailoPostProcessingStage
{
public:
	YoloInference(RPiCamApp *app);
	~YoloInference();

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	std::vector<Detection> runInference(const uint8_t *frame);
	void filterOutputObjects(std::vector<Detection> &objects);

	struct LtObject
	{
		Detection params;
		unsigned int visible;
		unsigned int hidden;
		bool matched;
	};

	std::vector<LtObject> lt_objects_;
	std::mutex lock_;
	PostProcessingLib postproc_;
	PostProcessingLib postproc_nms_;
	YoloParams *yolo_params_ = nullptr;

	// Config params
	std::string config_path_;
	std::string arch_;
	unsigned int max_detections_;
	float threshold_;
	bool temporal_filtering_;
	float tolerance_;
	float factor_;
	unsigned int visible_frames_;
	unsigned int hidden_frames_;
};

YoloInference::YoloInference(RPiCamApp *app)
	: HailoPostProcessingStage(app), postproc_(PostProcLibDir(POSTPROC_LIB)),
	  postproc_nms_(PostProcLibDir(POSTPROC_LIB_NMS))
{
}

YoloInference::~YoloInference()
{
	if (yolo_params_)
	{
		FreeFuncPtr free_func = reinterpret_cast<FreeFuncPtr>(postproc_.GetSymbol("free_resources"));
		if (free_func)
			free_func(yolo_params_);
	}
}

char const *YoloInference::Name() const
{
	return NAME;
}

void YoloInference::Read(boost::property_tree::ptree const &params)
{
	max_detections_ = params.get<unsigned int>("max_detections");
	threshold_ = params.get<float>("threshold", 0.5f);

	if (params.find("temporal_filter") != params.not_found())
	{
		temporal_filtering_ = true;
		tolerance_ = params.get<float>("temporal_filter.tolerance", 0.05);
		factor_ = params.get<float>("temporal_filter.factor", 0.2);
		visible_frames_ = params.get<unsigned int>("temporal_filter.visible_frames", 5);
		hidden_frames_ = params.get<unsigned int>("temporal_filter.hidden_frames", 2);
	}
	else
		temporal_filtering_ = false;

	InitFuncPtr init = reinterpret_cast<InitFuncPtr>(postproc_.GetSymbol("init"));
	const std::string config_file = params.get<std::string>("hailopp_config_file", "");
	if (init && !config_file.empty())
	{
		if (!fs::exists(config_file))
			throw std::runtime_error(std::string("hailo postprocess config file not found: ") + config_file);
		yolo_params_ = init(config_file, "");
	}

	HailoPostProcessingStage::Read(params);
}

void YoloInference::Configure()
{
	HailoPostProcessingStage::Configure();
}

bool YoloInference::Process(CompletedRequestPtr &completed_request)
{
	if (!HailoPostProcessingStage::Ready())
	{
		LOG_ERROR("HailoRT not ready!");
		return false;
	}

	if (low_res_info_.width != InputTensorSize().width || low_res_info_.height != InputTensorSize().height)
	{
		LOG_ERROR("Wrong low res size, expecting " << InputTensorSize().toString());
		return false;
	}

	BufferReadSync r(app_, completed_request->buffers[low_res_stream_]);
	libcamera::Span<uint8_t> buffer = r.Get()[0];
	std::shared_ptr<uint8_t> input;
	uint8_t *input_ptr;

	if (low_res_info_.pixel_format == libcamera::formats::YUV420)
	{
		StreamInfo rgb_info;
		rgb_info.width = InputTensorSize().width;
		rgb_info.height = InputTensorSize().height;
		rgb_info.stride = rgb_info.width * 3;

		input = allocator_.Allocate(rgb_info.stride * rgb_info.height);
		input_ptr = input.get();

		Yuv420ToRgb(input.get(), buffer.data(), low_res_info_, rgb_info);
	}
	else if (low_res_info_.pixel_format == libcamera::formats::RGB888 ||
			 low_res_info_.pixel_format == libcamera::formats::BGR888)
	{
		unsigned int stride = low_res_info_.width * 3;

		// If the stride shows we have padding on the right edge of the buffer, we must copy it out to another buffer
		// without padding.
		if (low_res_info_.stride != stride)
		{
			input = allocator_.Allocate(stride * low_res_info_.height);
			input_ptr = input.get();

			for (unsigned int i = 0; i < low_res_info_.height; i++)
				memcpy(input_ptr + i * stride, buffer.data() + i * low_res_info_.stride, stride);
		}
		else
			input_ptr = buffer.data();
	}
	else
	{
		LOG_ERROR("Unexpected lores format " << low_res_info_.pixel_format);
		return false;
	}

	std::vector<Detection> objects = runInference(input_ptr);
	if (objects.size())
	{
		if (temporal_filtering_)
		{
			// Process() can be concurrently called through different threads for consecutive CompletedRequests if
			// things are running behind.  So protect access to the inference state.
			std::scoped_lock<std::mutex> l(lock_);

			filterOutputObjects(objects);
			if (lt_objects_.size())
			{
				objects.clear();
				for (auto const &obj : lt_objects_)
				{
					if (!obj.hidden)
						objects.push_back(obj.params);
				}
			}
		}

		if (objects.size())
			completed_request->post_process_metadata.Set("object_detect.results", objects);
	}

	return false;
}

std::vector<Detection> YoloInference::runInference(const uint8_t *frame)
{
	hailort::AsyncInferJob job;
	std::vector<OutTensor> output_tensors;
	bool nms_on_hailo = false;
	hailo_status status;

	if (infer_model_->outputs().size() == 1 && infer_model_->outputs()[0].is_nms())
		nms_on_hailo = true;

	status = HailoPostProcessingStage::DispatchJob(frame, job, output_tensors);
	if (status != HAILO_SUCCESS)
		return {};

	// Prepare tensors for postprocessing.
	std::sort(output_tensors.begin(), output_tensors.end(), OutTensor::SortFunction);

	// Wait for job completion.
	status = job.wait(1s);
	if (status != HAILO_SUCCESS)
	{
		LOG_ERROR("Failed to wait for inference to finish, status = " << status);
		return {};
	}

	HailoROIPtr roi = MakeROI(output_tensors);

	if (nms_on_hailo)
	{
		PostProcFuncPtrNms filter = reinterpret_cast<PostProcFuncPtrNms>(postproc_nms_.GetSymbol("filter"));
		if (!filter)
			return {};

		filter(roi);
	}
	else
	{
		PostProcFuncPtr filter = reinterpret_cast<PostProcFuncPtr>(postproc_.GetSymbol("filter"));
		if (!filter)
			return {};

		filter(roi, yolo_params_);
	}

	std::vector<HailoDetectionPtr> detections = hailo_common::get_hailo_detections(roi);

	LOG(2, "------");

	// Translate results to the rpicam-apps Detection objects
	std::vector<Detection> results;
	Size output_size = output_stream_->configuration().size;
	for (auto const &d : detections)
	{
		if (d->get_confidence() < threshold_)
			continue;

		// Extract bounding box co-ordinates in the output image co-ordinates.
		auto const &box = d->get_bbox();
		int x0 = std::round(std::max(box.xmin(), 0.0f) * (output_size.width - 1));
		int x1 = std::round(std::min(box.xmax(), 1.0f) * (output_size.width - 1));
		int y0 = std::round(std::max(box.ymin(), 0.0f) * (output_size.height - 1));
		int y1 = std::round(std::min(box.ymax(), 1.0f) * (output_size.height - 1));
		results.emplace_back(d->get_class_id(), d->get_label(), d->get_confidence(), x0, y0,
							 x1 - x0, y1 - y0);
		LOG(2, "Object: " << results.back().toString());

		if (--max_detections_ == 0)
			break;
	}

	LOG(2, "------");

	return results;
}

void YoloInference::filterOutputObjects(std::vector<Detection> &objects)
{
	const Size isp_output_size = output_stream_->configuration().size;

	for (auto &lt_obj : lt_objects_)
		lt_obj.matched = false;

	for (auto const &object : objects)
	{
		bool matched = false;
		for (auto &lt_obj : lt_objects_)
		{
			// Try and match a detected object in our long term list.
			if (object.category == lt_obj.params.category &&
				std::abs(object.box.x - lt_obj.params.box.x) < tolerance_ * isp_output_size.width &&
				std::abs(object.box.y - lt_obj.params.box.y) < tolerance_ * isp_output_size.height &&
				std::abs((int)object.box.width - (int)lt_obj.params.box.width) < tolerance_ * isp_output_size.width &&
				std::abs((int)object.box.height - (int)lt_obj.params.box.height) < tolerance_ * isp_output_size.height)
			{
				lt_obj.matched = matched = true;
				lt_obj.params.confidence = object.confidence;
				lt_obj.params.box.x = factor_ * object.box.x + (1 - factor_) * lt_obj.params.box.x;
				lt_obj.params.box.y = factor_ * object.box.y + (1 - factor_) * lt_obj.params.box.y;
				lt_obj.params.box.width = factor_ * object.box.width + (1 - factor_) * lt_obj.params.box.width;
				lt_obj.params.box.height = factor_ * object.box.height + (1 - factor_) * lt_obj.params.box.height;
				// Reset the visibility counter for when the object next disappears.
				lt_obj.visible = visible_frames_;
				// Decrement the hidden counter until the object becomes visible in the list.
				lt_obj.hidden = std::max(0, (int)lt_obj.hidden - 1);
				break;
			}
		}

		// Add the object to the long term list if not found.  This object will remain hidden for hidden_frames_
		// consecutive frames.
		if (!matched)
			lt_objects_.push_back({ object, visible_frames_, hidden_frames_, 1 });
	}

	for (auto &lt_obj : lt_objects_)
	{
		if (!lt_obj.matched)
		{
			// If a non matched object in the long term list is still hidden, set visible count to 0 so that it must be
			// matched for hidden_frames_ consecutive frames before becoming visible. Otherwise, decrement the visible
			// count of unmatched objects in the long term list.
			if (lt_obj.hidden)
				lt_obj.visible = 0;
			else
				lt_obj.visible--;
		}
	}

	// Remove now invisible objects from the long term list.
	lt_objects_.erase(std::remove_if(lt_objects_.begin(), lt_objects_.end(),
									 [](const LtObject &obj) { return !obj.matched && !obj.visible; }),
					  lt_objects_.end());
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new YoloInference(app);
}

static RegisterStage reg(NAME, &Create);
