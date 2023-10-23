/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * tf_stage.hpp - base class for TensorFlowLite stages
 */
#include "tf_stage.hpp"

TfStage::TfStage(LibcameraApp *app, int tf_w, int tf_h) : PostProcessingStage(app), tf_w_(tf_w), tf_h_(tf_h)
{
	if (tf_w_ <= 0 || tf_h_ <= 0)
		throw std::runtime_error("TfStage: Bad TFLite input dimensions");
}

void TfStage::Read(boost::property_tree::ptree const &params)
{
	config_->number_of_threads = params.get<int>("number_of_threads", 2);
	config_->refresh_rate = params.get<int>("refresh_rate", 5);
	config_->model_file = params.get<std::string>("model_file", "");
	config_->verbose = params.get<int>("verbose", 0);
	config_->normalisation_offset = params.get<float>("normalisation_offset", 127.5);
	config_->normalisation_scale = params.get<float>("normalisation_scale", 127.5);

	initialise();

	readExtras(params);
}

void TfStage::initialise()
{
	model_ = tflite::FlatBufferModel::BuildFromFile(config_->model_file.c_str());
	if (!model_)
		throw std::runtime_error("TfStage: Failed to load model");
	LOG(1, "TfStage: Loaded model " << config_->model_file);

	tflite::ops::builtin::BuiltinOpResolver resolver;
	tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
	if (!interpreter_)
		throw std::runtime_error("TfStage: Failed to construct interpreter");

	if (config_->number_of_threads != -1)
		interpreter_->SetNumThreads(config_->number_of_threads);

	if (interpreter_->AllocateTensors() != kTfLiteOk)
		throw std::runtime_error("TfStage: Failed to allocate tensors");

	// Make an attempt to verify that the model expects this size of input.
	int input = interpreter_->inputs()[0];
	size_t size = interpreter_->tensor(input)->bytes;
	size_t check = tf_w_ * tf_h_ * 3; // assume RGB
	if (interpreter_->tensor(input)->type == kTfLiteUInt8)
		check *= sizeof(uint8_t);
	else if (interpreter_->tensor(input)->type == kTfLiteFloat32)
		check *= sizeof(float);
	else
		throw std::runtime_error("TfStage: Input tensor data type not supported");

	// Causes might include loading the wrong model.
	if (check != size)
		throw std::runtime_error("TfStage: Input tensor size mismatch");
}

void TfStage::Configure()
{
	lores_stream_ = app_->LoresStream();
	if (lores_stream_)
	{
		lores_info_ = app_->GetStreamInfo(lores_stream_);
		if (config_->verbose)
			LOG(1, "TfStage: Low resolution stream is " << lores_info_.width << "x" << lores_info_.height);
		if (tf_w_ > lores_info_.width || tf_h_ > lores_info_.height)
		{
			LOG_ERROR("TfStage: WARNING: Low resolution image too small");
			lores_stream_ = nullptr;
		}
	}
	else if (config_->verbose)
		LOG(1, "TfStage: no low resolution stream");

	main_stream_ = app_->GetMainStream();
	if (main_stream_)
	{
		main_stream_info_ = app_->GetStreamInfo(main_stream_);
		if (config_->verbose)
			LOG(1, "TfStage: Main stream is " << main_stream_info_.width << "x" << main_stream_info_.height);
	}
	else if (config_->verbose)
		LOG(1, "TfStage: No main stream");

	checkConfiguration();
}

bool TfStage::Process(CompletedRequestPtr &completed_request)
{
	if (!lores_stream_)
		return false;

	{
		std::unique_lock<std::mutex> lck(future_mutex_);
		if (config_->refresh_rate && completed_request->sequence % config_->refresh_rate == 0 &&
			(!future_ || future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready))
		{
			BufferReadSync r(app_, completed_request->buffers[lores_stream_]);
			libcamera::Span<uint8_t> buffer = r.Get()[0];

			// Copy the lores image here and let the asynchronous thread convert it to RGB.
			// Doing the "extra" copy is in fact hugely beneficial because it turns uncacned
			// memory into cached memory, which is then *much* quicker.
			lores_copy_.assign(buffer.data(), buffer.data() + buffer.size());

			future_ = std::make_unique<std::future<void>>();
			*future_ = std::async(std::launch::async, [this] {
				auto time_taken = ExecutionTime<std::micro>(&TfStage::runInference, this).count();

				if (config_->verbose)
					LOG(1, "TfStage: Inference time: " << time_taken << " ms");
			});
		}
	}

	std::unique_lock<std::mutex> lock(output_mutex_);
	applyResults(completed_request);

	return false;
}

void TfStage::runInference()
{
	int input = interpreter_->inputs()[0];
	StreamInfo tf_info;
	tf_info.width = tf_w_, tf_info.height = tf_h_, tf_info.stride = tf_w_ * 3;
	std::vector<uint8_t> rgb_image = Yuv420ToRgb(lores_copy_.data(), lores_info_, tf_info);

	if (interpreter_->tensor(input)->type == kTfLiteUInt8)
	{
		uint8_t *tensor = interpreter_->typed_tensor<uint8_t>(input);
		for (unsigned int i = 0; i < rgb_image.size(); i++)
			tensor[i] = rgb_image[i];
	}
	else if (interpreter_->tensor(input)->type == kTfLiteFloat32)
	{
		float *tensor = interpreter_->typed_tensor<float>(input);
		for (unsigned int i = 0; i < rgb_image.size(); i++)
			tensor[i] = (rgb_image[i] - config_->normalisation_offset) / config_->normalisation_scale;
	}

	if (interpreter_->Invoke() != kTfLiteOk)
		throw std::runtime_error("TfStage: Failed to invoke TFLite");

	std::unique_lock<std::mutex> lock(output_mutex_);
	interpretOutputs();
}

void TfStage::Stop()
{
	if (future_)
		future_->wait();
}
