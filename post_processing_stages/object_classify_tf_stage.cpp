/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_classify_tf_stage - object classifier
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

using Stream = libcamera::Stream;

constexpr int IMAGE_WIDTH = 224;
constexpr int IMAGE_HEIGHT = 224;
constexpr int IMAGE_CHANNELS = 3;

class ObjectClassifyTfStage : public PostProcessingStage
{
public:
	ObjectClassifyTfStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params);

	void Configure();

	bool Process(CompletedRequest &completed_request);

	void Stop();

private:
	void initialize();

	void runInference();

	void readLabelsFile(const std::string &file_name, std::vector<std::string> *result, size_t *found_label_count);

	std::vector<uint8_t> yuvToRgb(uint8_t *mem, int w, int h, int stride);

	void getTopResults(uint8_t *prediction, int prediction_size, size_t num_results);

	struct Config
	{
		int number_of_threads;
		int number_of_results;
		int refresh_rate;
		int top_n_results;
		float threshold_high;
		float threshold_low;
		std::string model_file;
		std::string labels_file;
		int display_labels;
		int verbose;
	} config_;
	Stream *stream_;
	int w_, h_, stride_;
	std::mutex future_ptr_mutex_;
	std::mutex output_results_mutex_;
	std::unique_ptr<tflite::FlatBufferModel> model_;
	std::unique_ptr<tflite::Interpreter> interpreter_;
	std::vector<std::pair<std::string, float>> output_results_;
	std::unique_ptr<std::future<void>> future_ptr_;
	std::vector<std::string> labels_;
	size_t label_count_;
	std::vector<uint8_t> tensor_input_;
	std::vector<std::pair<float, int>> top_results_;
};

#define NAME "object_classify_tf"

char const *ObjectClassifyTfStage::Name() const
{
	return NAME;
}

void ObjectClassifyTfStage::Read(boost::property_tree::ptree const &params)
{
	config_.number_of_results = params.get<int>("number_of_results", 3);
	config_.number_of_threads = params.get<int>("number_of_threads", 2);
	config_.refresh_rate = params.get<int>("refresh_rate", 10);
	config_.threshold_high = params.get<float>("threshold_high", 0.2f);
	config_.threshold_low = params.get<float>("threshold_low", 0.1f);
	config_.model_file = params.get<std::string>("model_file", "/home/pi/models/mobilenet_quant_v1_224.tflite");
	config_.labels_file = params.get<std::string>("labels_file", "/home/pi/models/labels.txt");
	config_.display_labels = params.get<int>("display_labels", 1);
	config_.verbose = params.get<int>("verbose", 1);

	initialize();
}

void ObjectClassifyTfStage::initialize()
{
	model_ = tflite::FlatBufferModel::BuildFromFile(config_.model_file.c_str());

	std::cout << "Loaded model " << config_.model_file << std::endl;
	model_->error_reporter();

	tflite::ops::builtin::BuiltinOpResolver resolver;

	tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
	if (!interpreter_)
		throw std::runtime_error("ObjectClassifyStage: Failed to construct interpreter");

	std::cout << "Input size " << interpreter_->inputs().size() << std::endl;

	if (config_.number_of_threads != -1)
		interpreter_->SetNumThreads(config_.number_of_threads);

	if (interpreter_->AllocateTensors() != kTfLiteOk)
		throw std::runtime_error("ObjectClassifyStage: Failed to allocate tensors");

	readLabelsFile(config_.labels_file, &labels_, &label_count_);
}

void ObjectClassifyTfStage::Configure()
{
	stream_ = app_->LoresStream();
	if (stream_)
	{
		app_->StreamDimensions(stream_, &w_, &h_, &stride_);
		if (w_ != IMAGE_WIDTH || h_ != IMAGE_HEIGHT)
			throw std::runtime_error("ObjectClassifyStage: Lores width and height must be 224");
	}
}

bool ObjectClassifyTfStage::Process(CompletedRequest &completed_request)
{
	if (!stream_)
		return false;

	{
		std::unique_lock<std::mutex> lck(future_ptr_mutex_);
		if (config_.refresh_rate && completed_request.sequence % config_.refresh_rate == 0 &&
			(!future_ptr_ || future_ptr_->wait_for(std::chrono::seconds(0)) == std::future_status::ready))
		{
			libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request.buffers[stream_])[0];
			uint8_t *ptr = buffer.data();

			tensor_input_ = yuvToRgb(ptr, w_, h_, stride_);

			future_ptr_ = std::make_unique<std::future<void>>();
			*future_ptr_ = std::move(std::async(std::launch::async, [this] {
				auto time_taken =
					ExecutionTime<std::micro>(std::bind(&ObjectClassifyTfStage::runInference, this)).count();

				if (config_.verbose)
					std::cout << "Detection Time: " << time_taken << " ms" << std::endl;
			}));
		}
	}

	std::unique_lock<std::mutex> lock(output_results_mutex_);

	completed_request.post_process_metadata.Set("object_classify.results", output_results_);

	if (config_.display_labels)
	{
		std::stringstream annotation;
		annotation.precision(2);
		annotation << "Detected: ";
		bool first = true;

		for (const auto &result : output_results_)
		{
			unsigned int start = result.first.find(':');
			unsigned int end = result.first.find(',');
			// Note: this does the right thing if start and/or end are std::string::npos
			std::string label = result.first.substr(start + 1, end - (start + 1));
			if (!first)
				annotation << ", ";
			annotation << label << " " << result.second;
			first = false;
		}

		completed_request.post_process_metadata.Set("annotate.text", annotation.str());
	}

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new ObjectClassifyTfStage(app);
}

static RegisterStage reg(NAME, &Create);

void ObjectClassifyTfStage::runInference()
{
	int input = interpreter_->inputs()[0];

	TfLiteIntArray *dims = interpreter_->tensor(input)->dims;

	for (int i = 0; i < tensor_input_.size(); i++)
		interpreter_->typed_tensor<uint8_t>(input)[i] = static_cast<uint8_t>(tensor_input_[i]);

	if (interpreter_->Invoke() != kTfLiteOk)
		throw std::runtime_error("ObjectClassifyStage: Failed to invoke TFLite");

	int output = interpreter_->outputs()[0];
	TfLiteIntArray *output_dims = interpreter_->tensor(output)->dims;
	// assume output dims to be something like (1, 1, ... ,size)
	auto output_size = output_dims->data[output_dims->size - 1];

	getTopResults(interpreter_->typed_output_tensor<uint8_t>(0), output_size, config_.number_of_results);

	std::unique_lock<std::mutex> lock(output_results_mutex_);
	output_results_.clear();

	for (const auto &result : top_results_)
	{
		float confidence = result.first;
		int index = result.second;
		output_results_.push_back(std::make_pair(labels_[index], confidence));
	}

	if (config_.verbose)
	{
		for (const auto &result : output_results_)
			std::cout << result.first << " : " << std::to_string(result.second) << std::endl;
		std::cout << std::endl;
	}
}

void ObjectClassifyTfStage::readLabelsFile(const std::string &file_name, std::vector<std::string> *result,
												   size_t *found_label_count)
{
	std::ifstream file(file_name);
	if (!file)
		throw std::runtime_error("ObjectClassifyStage: Failed to load labels file");

	result->clear();
	std::string line;
	while (std::getline(file, line))
		result->push_back(line);

	*found_label_count = result->size();
	constexpr int padding = 16;

	while (result->size() % padding)
		result->emplace_back();
}

std::vector<uint8_t> ObjectClassifyTfStage::yuvToRgb(uint8_t *mem, int w, int h, int stride)
{
	int dst;
	std::vector<uint8_t> output(h * w * 3);

	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			uint8_t *Y = mem + y * stride + x;
			uint8_t *U = mem + (h * stride) + (y / 2) * (stride / 2) + (x / 2);
			uint8_t *V = U + (h * stride / 4);
			dst = (x + y * w) * 3;

			int YY = *Y, UU = *U, VV = *V;

			output[dst] = std::clamp<int>((YY + (1.402 * (VV - 128))), 0, 255);
			output[dst + 1] = std::clamp<int>((YY - 0.345 * (UU - 128) - 0.714 * (VV - 128)), 0, 255);
			output[dst + 2] = std::clamp<int>((YY + 1.771 * (UU - 128)), 0, 255);
		}
	}

	return output;
}

void ObjectClassifyTfStage::getTopResults(uint8_t *prediction, int prediction_size, size_t num_results)
{
	// Will contain top N results in ascending order.
	std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>>
		top_result_pq;
	std::vector<std::pair<float, int>> top_results_old = std::move(top_results_);
	top_results_.clear();

	for (int i = 0; i < prediction_size; ++i)
	{
		float confidence = prediction[i] / 255.0;

		if (confidence < config_.threshold_low)
			continue;

		// Consider keeping if above the high threshold or it was in the old list.
		if (confidence >= config_.threshold_high ||
			std::find_if(top_results_old.begin(), top_results_old.end(),
						 [i] (auto &p) { return p.second == i; }) != top_results_old.end())
		{
			top_result_pq.push(std::pair<float, int>(confidence, i));

			// If at capacity, kick the smallest value out.
			if (top_result_pq.size() > num_results)
				top_result_pq.pop();
		}
	}

	// Copy to output vector and reverse into descending order.
	while (!top_result_pq.empty())
	{
		top_results_.push_back(top_result_pq.top());
		top_result_pq.pop();
	}
	std::reverse(top_results_.begin(), top_results_.end());
}

void ObjectClassifyTfStage::Stop()
{
	if (future_ptr_)
		future_ptr_->wait();
}
