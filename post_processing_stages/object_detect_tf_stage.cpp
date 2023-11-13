/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_detect_tf_stage.cpp - object detector
 */

#include "object_detect.hpp"
#include "tf_stage.hpp"

using Rectangle = libcamera::Rectangle;

constexpr int WIDTH = 300;
constexpr int HEIGHT = 300;

struct ObjectDetectTfConfig : public TfConfig
{
	float confidence_threshold;
	float overlap_threshold;
};

#define NAME "object_detect_tf"

class ObjectDetectTfStage : public TfStage
{
public:
	// The model we use expects 224x224 images.
	ObjectDetectTfStage(RPiCamApp *app) : TfStage(app, WIDTH, HEIGHT)
	{
		config_ = std::make_unique<ObjectDetectTfConfig>();
	}
	char const *Name() const override { return NAME; }

protected:
	ObjectDetectTfConfig *config() const { return static_cast<ObjectDetectTfConfig *>(config_.get()); }

	// Read the label file, plus some thresholds.
	void readExtras(boost::property_tree::ptree const &params) override;

	void checkConfiguration() override;

	// Retrieve the top-n most likely results.
	void interpretOutputs() override;

	// Attach the results as metadata; optionally write the labels too for the annotate_cv
	// stage to pick up.
	void applyResults(CompletedRequestPtr &completed_request) override;

private:
	void readLabelsFile(const std::string &file_name);

	std::vector<Detection> output_results_;
	std::vector<std::string> labels_;
	size_t label_count_;
};

void ObjectDetectTfStage::readExtras(boost::property_tree::ptree const &params)
{
	config()->confidence_threshold = params.get<float>("confidence_threshold", 0.5f);
	config()->overlap_threshold = params.get<float>("overlap_threshold", 0.5f);

	std::string labels_file = params.get<std::string>("labels_file", "");
	readLabelsFile(labels_file);
	if (config()->verbose)
		LOG(1, "Read " << label_count_ << " labels");

	// Check the tensor outputs and label classes match up.
	int output = interpreter_->outputs()[0];
	TfLiteIntArray *output_dims = interpreter_->tensor(output)->dims;
	// Causes might include loading the wrong model.
	if (output_dims->size != 3 || output_dims->data[0] != 1 || output_dims->data[1] != 10 || output_dims->data[2] != 4)
		throw std::runtime_error("ObjectDetectTfStage: unexpected output dimensions");
}

void ObjectDetectTfStage::readLabelsFile(const std::string &file_name)
{
	std::ifstream file(file_name);
	if (!file)
		throw std::runtime_error("ObjectDetectTfStage: Failed to load labels file");

	std::string line;
	std::getline(file, line); // discard first line of ???
	while (std::getline(file, line))
		labels_.push_back(line);

	label_count_ = labels_.size();
}

void ObjectDetectTfStage::checkConfiguration()
{
	if (!main_stream_)
		throw std::runtime_error("ObjectDetectTfStage: Main stream is required");
}

void ObjectDetectTfStage::applyResults(CompletedRequestPtr &completed_request)
{
	completed_request->post_process_metadata.Set("object_detect.results", output_results_);
}

static unsigned int area(const Rectangle &r)
{
	return r.width * r.height;
}

void ObjectDetectTfStage::interpretOutputs()
{
	int box_index = interpreter_->outputs()[0];
	int class_index = interpreter_->outputs()[1];
	int score_index = interpreter_->outputs()[2];
	int num_detections = interpreter_->tensor(box_index)->dims->data[1];
	float *boxes = interpreter_->tensor(box_index)->data.f;
	float *scores = interpreter_->tensor(score_index)->data.f;
	float *classes = interpreter_->tensor(class_index)->data.f;

	output_results_.clear();

	for (int i = 0; i < num_detections; i++)
	{
		if (scores[i] < config()->confidence_threshold)
			continue;

		// The coords in the WIDTH x HEIGHT image fed to the network are:
		int y = std::clamp<int>(HEIGHT * boxes[i * 4 + 0], 0, HEIGHT);
		int x = std::clamp<int>(WIDTH * boxes[i * 4 + 1], 0, WIDTH);
		int h = std::clamp<int>(HEIGHT * boxes[i * 4 + 2] - y, 0, HEIGHT);
		int w = std::clamp<int>(WIDTH * boxes[i * 4 + 3] - x, 0, WIDTH);
		// The network is fed a crop from the lores (if that was too large), so the coords
		// in the full lores image are:
		y += (lores_info_.height - HEIGHT) / 2;
		x += (lores_info_.width - WIDTH) / 2;
		// The lores is a pure scaling of the main image (squishing if the aspect ratios
		// don't match), so:
		y = y * main_stream_info_.height / lores_info_.height;
		x = x * main_stream_info_.width / lores_info_.width;
		h = h * main_stream_info_.height / lores_info_.height;
		w = w * main_stream_info_.width / lores_info_.width;

		int c = classes[i];
		Detection detection(c, labels_[c], scores[i], x, y, w, h);

		// Before adding this detection to the results, see if it overlaps an existing one.
		bool overlapped = false;
		for (auto &prev_detection : output_results_)
		{
			if (prev_detection.category == c)
			{
				unsigned int prev_area = area(prev_detection.box);
				unsigned int new_area = area(detection.box);
				unsigned int overlap = area(prev_detection.box.boundedTo(detection.box));
				if (overlap > config()->overlap_threshold * prev_area ||
					overlap > config()->overlap_threshold * new_area)
				{
					// Take the box with the higher confidence.
					if (detection.confidence > prev_detection.confidence)
						prev_detection = detection;
					overlapped = true;
					break;
				}
			}
		}
		if (!overlapped)
			output_results_.push_back(detection);
	}

	if (config()->verbose)
	{
		for (auto &detection : output_results_)
			LOG(1, detection.toString());
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectDetectTfStage(app);
}

static RegisterStage reg(NAME, &Create);
