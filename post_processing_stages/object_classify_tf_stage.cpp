/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_classify_tf_stage.cpp - object classifier
 */

#include "tf_stage.hpp"

struct ObjectClassifyTfConfig : public TfConfig
{
	int number_of_results;
	int top_n_results;
	float threshold_high;
	float threshold_low;
	bool display_labels;
};

#define NAME "object_classify_tf"

class ObjectClassifyTfStage : public TfStage
{
public:
	// The model we use expects 224x224 images.
	ObjectClassifyTfStage(RPiCamApp *app) : TfStage(app, 224, 224)
	{
		config_ = std::make_unique<ObjectClassifyTfConfig>();
	}
	char const *Name() const override { return NAME; }

protected:
	ObjectClassifyTfConfig *config() const { return static_cast<ObjectClassifyTfConfig *>(config_.get()); }

	// Read the label file, plus some confidence thresholds.
	void readExtras(boost::property_tree::ptree const &params) override;

	// Retrieve the top-n most likely results.
	void interpretOutputs() override;

	// Attach the results as metadata; optionally write the labels too for the annotate_cv
	// stage to pick up.
	void applyResults(CompletedRequestPtr &completed_request) override;

private:
	void readLabelsFile(const std::string &file_name);
	void getTopResults(uint8_t *prediction, int prediction_size, size_t num_results);

	std::vector<std::pair<std::string, float>> output_results_;
	std::vector<std::string> labels_;
	size_t label_count_;
	std::vector<std::pair<float, int>> top_results_;
};

void ObjectClassifyTfStage::readExtras(boost::property_tree::ptree const &params)
{
	config()->number_of_results = params.get<int>("number_of_results", 3);
	config()->threshold_high = params.get<float>("threshold_high", 0.2f);
	config()->threshold_low = params.get<float>("threshold_low", 0.1f);
	config()->display_labels = params.get<int>("display_labels", 1);

	std::string labels_file = params.get<std::string>("labels_file", "/home/pi/models/labels.txt");
	readLabelsFile(labels_file);

	// Check the tensor outputs and label classes match up.
	int output = interpreter_->outputs()[0];
	TfLiteIntArray *output_dims = interpreter_->tensor(output)->dims;
	// Causes might include loading the wrong model, or the wrong labels file.
	if (output_dims->data[output_dims->size - 1] != static_cast<int>(label_count_))
		throw std::runtime_error("ObjectClassifyTfStage: Label count mismatch");
}

void ObjectClassifyTfStage::readLabelsFile(const std::string &file_name)
{
	std::ifstream file(file_name);
	if (!file)
		throw std::runtime_error("ObjectClassifyTfStage: Failed to load labels file");

	std::string line;
	while (std::getline(file, line))
		labels_.push_back(line);

	label_count_ = labels_.size();

	constexpr int padding = 16;
	while (labels_.size() % padding)
		labels_.emplace_back();
}

void ObjectClassifyTfStage::applyResults(CompletedRequestPtr &completed_request)
{
	completed_request->post_process_metadata.Set("object_classify.results", output_results_);

	if (config()->display_labels)
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

		completed_request->post_process_metadata.Set("annotate.text", annotation.str());
	}
}

void ObjectClassifyTfStage::interpretOutputs()
{
	int output = interpreter_->outputs()[0];
	TfLiteIntArray *output_dims = interpreter_->tensor(output)->dims;
	// assume output dims to be something like (1, 1, ... ,size)
	auto output_size = output_dims->data[output_dims->size - 1];

	getTopResults(interpreter_->typed_output_tensor<uint8_t>(0), output_size, config()->number_of_results);

	output_results_.clear();

	for (const auto &result : top_results_)
	{
		float confidence = result.first;
		int index = result.second;
		output_results_.push_back(std::make_pair(labels_[index], confidence));
	}

	if (config_->verbose)
	{
		for (const auto &result : output_results_)
			LOG(1, result.first << " : " << std::to_string(result.second));
		LOG(1, "");
	}
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

		if (confidence < config()->threshold_low)
			continue;

		// Consider keeping if above the high threshold or it was in the old list.
		if (confidence >= config()->threshold_high ||
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

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectClassifyTfStage(app);
}

static RegisterStage reg(NAME, &Create);
