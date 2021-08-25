/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * pose_estimation_tf_stage - pose estimator
 */

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"
#include "core/post_processing_stage.hpp"

#include "tensorflow/lite/builtin_op_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"

using Stream = libcamera::Stream;

struct TfSettings
{
	std::string model_name;
	int number_of_threads;
	int refresh_rate;
};

constexpr int IMAGE_WIDTH = 258;
constexpr int IMAGE_HEIGHT = 258;
constexpr int IMAGE_CHANNELS = 3;
constexpr int FEATURE_SIZE = 17;
constexpr int HEATMAP_DIMS = 9;

class PoseEstimationTfStage : public PostProcessingStage
{
public:
	PoseEstimationTfStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params);

	void Configure();

	bool Process(CompletedRequest &completed_request);

	void Stop();

private:
	void initialize();

	void runInference();

	std::vector<float> yuvToRgb(uint8_t *mem, int w, int h, int stride);

	Stream *lores_stream_;
	Stream *main_stream_;
	int w_, h_, stride_;
	int main_w_, main_h_, main_stride_;
	TfSettings settings_;
	std::unique_ptr<tflite::FlatBufferModel> model_;
	std::unique_ptr<tflite::Interpreter> interpreter_;
	std::unique_ptr<std::future<void>> future_ptr_;
	std::mutex future_ptr_mutex_;
	std::mutex output_results_mutex_;
	std::vector<float> tensor_input_;
	std::vector<libcamera::Point> heats_;
	std::vector<float> confidences_;
	std::vector<libcamera::Point> locations_;
};

#define NAME "pose_estimation_tf"

char const *PoseEstimationTfStage::Name() const
{
	return NAME;
}

void PoseEstimationTfStage::Read(boost::property_tree::ptree const &params)
{
	settings_.refresh_rate = params.get<int>("refresh_rate", 10);
	settings_.model_name = params.get<std::string>(
		"model_file", "/home/pi/models/posenet_mobilenet_v1_100_257x257_multi_kpt_stripped.tflite");
	settings_.number_of_threads = params.get<int>("number_of_threads", 3);

	initialize();
}

void PoseEstimationTfStage::initialize()
{
	model_ = tflite::FlatBufferModel::BuildFromFile(settings_.model_name.c_str());
	std::cout << "Loaded model " << settings_.model_name << std::endl;

	model_->error_reporter();
	tflite::ops::builtin::BuiltinOpResolver resolver;

	tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
	if (!interpreter_)
		throw std::runtime_error("Failed to construct interpreter");

	std::cout << "Input size " << interpreter_->inputs().size() << std::endl;

	if (settings_.number_of_threads != -1)
		interpreter_->SetNumThreads(settings_.number_of_threads);

	if (interpreter_->AllocateTensors() != kTfLiteOk)
		throw std::runtime_error("Failed to allocate tensors");
}

void PoseEstimationTfStage::Configure()
{
	lores_stream_ = app_->LoresStream();
	if (lores_stream_)
	{
		app_->StreamDimensions(lores_stream_, &w_, &h_, &stride_);
		std::cout << w_ << " " << h_ << std::endl;
		if (w_ != IMAGE_WIDTH || h_ != IMAGE_HEIGHT)
			throw std::runtime_error("Lores width and height must be 258");
	}
	main_stream_ = app_->GetMainStream();
	if (main_stream_)
	{
		app_->StreamDimensions(main_stream_, &main_w_, &main_h_, &main_stride_);
		std::cout << main_w_ << " " << main_h_ << std::endl;
	}
	else
	{
		throw std::runtime_error("Main Stream is null");
	}
}

bool PoseEstimationTfStage::Process(CompletedRequest &completed_request)
{
	if (!lores_stream_)
		return false;
	{
		std::unique_lock<std::mutex> lck(future_ptr_mutex_);
		if (completed_request.sequence % settings_.refresh_rate == 0 &&
			(!future_ptr_ || future_ptr_->wait_for(std::chrono::seconds(0)) == std::future_status::ready))
		{
			libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request.buffers[lores_stream_])[0];
			uint8_t *ptr = buffer.data();

			tensor_input_ = yuvToRgb(ptr, w_, h_, stride_);

			future_ptr_ = std::make_unique<std::future<void>>();
			*future_ptr_ = std::move(std::async(std::launch::async, [this] { runInference(); }));
		}
	}

	std::unique_lock<std::mutex> lock(output_results_mutex_);
	completed_request.post_process_metadata.Set("pose_estimation.locations", locations_);
	completed_request.post_process_metadata.Set("pose_estimation.confidences", confidences_);

	return false;
}

void PoseEstimationTfStage::runInference()
{
	// This code has been adapted from the "Qengineering/TensorFlow_Lite_Pose_RPi_32-bits" repository and can be
	// found here: "https://github.com/Qengineering/TensorFlow_Lite_Pose_RPi_32-bits/blob/master/Pose_single.cpp"
	for (int i = 0; i < tensor_input_.size(); i++)
	{
		interpreter_->typed_tensor<float>(interpreter_->inputs()[0])[i] = static_cast<float>(tensor_input_[i]);
	}

	interpreter_->Invoke();

	std::unique_lock<std::mutex> lock(output_results_mutex_);

	float *heatmaps = interpreter_->tensor(interpreter_->outputs()[0])->data.f;
	float *offsets = interpreter_->tensor(interpreter_->outputs()[1])->data.f;

	float confidence_temp;
	int j;
	heats_.clear();

	for (int i = 0; i < FEATURE_SIZE; i++)
	{
		confidence_temp = heatmaps[i];
		libcamera::Point heat_coord;
		for (int y = 0; y < HEATMAP_DIMS; y++)
		{
			for (int x = 0; x < HEATMAP_DIMS; x++)
			{
				j = FEATURE_SIZE * (HEATMAP_DIMS * y + x) + i;
				if (heatmaps[j] > confidence_temp)
				{
					confidence_temp = heatmaps[j];
					heat_coord.x = x;
					heat_coord.y = y;
				}
			}
		}
		heats_.push_back(heat_coord);
		confidences_.push_back(confidence_temp);
	}

	locations_.clear();

	for (int i = 0; i < FEATURE_SIZE; i++)
	{
		libcamera::Point location_coord;
		int x = heats_[i].x, y = heats_[i].y, j = (FEATURE_SIZE * 2) * (HEATMAP_DIMS * y + x) + i;

		location_coord.y = (y * main_h_) / (HEATMAP_DIMS - 1) + offsets[j];
		location_coord.x = (x * main_w_) / (HEATMAP_DIMS - 1) + offsets[j + FEATURE_SIZE];

		locations_.push_back(location_coord);
	}
}

std::vector<float> PoseEstimationTfStage::yuvToRgb(uint8_t *mem, int w, int h, int stride)
{
	int dst;
	//our Low-Res input (YUV420) has to be 258*258 but the PoseNet model requires 257*257 so we trim the ends.
	std::vector<float> output((h - 1) * (w - 1) * 3);

	for (int y = 0; y < h - 1; y++)
	{
		for (int x = 0; x < w - 1; x++)
		{
			uint8_t *Y = mem + y * stride + x;
			uint8_t *U = mem + (h * stride) + ((int)floor(y / 2)) * (stride / 2) + ((int)floor(x / 2));
			uint8_t *V = mem + (5 * h * stride) / 4 + ((int)floor(y / 2)) * (stride / 2) + ((int)floor(x / 2));
			dst = (x + y * (w - 1)) * 3;

			int YY = *Y, UU = *U, VV = *V;

			output[dst] = (std::clamp<int>((YY + (1.402 * (VV - 128))), 0, 255) - 127.5) / 127.5;
			output[dst + 1] = (std::clamp<int>((YY - 0.345 * (UU - 128) - 0.714 * (VV - 128)), 0, 255) - 127.5) / 127.5;
			output[dst + 2] = (std::clamp<int>((YY + 1.771 * (UU - 128)), 0, 255) - 127.5) / 127.5;
		}
	}

	return output;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new PoseEstimationTfStage(app);
}

static RegisterStage reg(NAME, &Create);

void PoseEstimationTfStage::Stop()
{
	if (future_ptr_)
		future_ptr_->wait();
}
