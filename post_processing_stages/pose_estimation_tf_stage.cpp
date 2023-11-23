/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * pose_estimation_tf_stage - pose estimator
 */

#include "tf_stage.hpp"

constexpr int FEATURE_SIZE = 17;
constexpr int HEATMAP_DIMS = 9;

#define NAME "pose_estimation_tf"

class PoseEstimationTfStage : public TfStage
{
public:
	// The model we use expects 257x257 images. Really.
	PoseEstimationTfStage(RPiCamApp *app) : TfStage(app, 257, 257) { config_ = std::make_unique<TfConfig>(); }
	char const *Name() const override { return NAME; }

protected:
	void readExtras(boost::property_tree::ptree const &params) override;

	void checkConfiguration() override;

	// Retrieve the various joint coordinates and confidences from the model.
	void interpretOutputs() override;

	// Attach results as metadata.
	void applyResults(CompletedRequestPtr &completed_request) override;

private:
	std::vector<libcamera::Point> heats_;
	std::vector<float> confidences_;
	std::vector<libcamera::Point> locations_;
};

void PoseEstimationTfStage::readExtras([[maybe_unused]] boost::property_tree::ptree const &params)
{
	// Actually we don't read anything, but we can check the output tensor dimensions.
	int output = interpreter_->outputs()[0];
	TfLiteIntArray *dims = interpreter_->tensor(output)->dims;
	// Causes might include loading the wrong model.
	if (dims->data[0] != 1 || dims->data[1] != HEATMAP_DIMS || dims->data[2] != HEATMAP_DIMS ||
		dims->data[3] != FEATURE_SIZE)
		throw std::runtime_error("PoseEstimationTfStage: Unexpected output dimensions");
}

void PoseEstimationTfStage::checkConfiguration()
{
	if (!main_stream_)
		throw std::runtime_error("PoseEstimationTfStage: Main stream is required");
}

void PoseEstimationTfStage::applyResults(CompletedRequestPtr &completed_request)
{
	completed_request->post_process_metadata.Set("pose_estimation.locations", locations_);
	completed_request->post_process_metadata.Set("pose_estimation.confidences", confidences_);
}

void PoseEstimationTfStage::interpretOutputs()
{
	// This code has been adapted from the "Qengineering/TensorFlow_Lite_Pose_RPi_32-bits" repository and can be
	// found here: "https://github.com/Qengineering/TensorFlow_Lite_Pose_RPi_32-bits/blob/master/Pose_single.cpp"
	float *heatmaps = interpreter_->tensor(interpreter_->outputs()[0])->data.f;
	float *offsets = interpreter_->tensor(interpreter_->outputs()[1])->data.f;

	heats_.clear();
	confidences_.clear();
	locations_.clear();

	for (int i = 0; i < FEATURE_SIZE; i++)
	{
		float confidence_temp = heatmaps[i];
		libcamera::Point heat_coord;
		for (int y = 0; y < HEATMAP_DIMS; y++)
		{
			for (int x = 0; x < HEATMAP_DIMS; x++)
			{
				int j = FEATURE_SIZE * (HEATMAP_DIMS * y + x) + i;
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

	for (int i = 0; i < FEATURE_SIZE; i++)
	{
		libcamera::Point location_coord;
		int x = heats_[i].x, y = heats_[i].y, j = (FEATURE_SIZE * 2) * (HEATMAP_DIMS * y + x) + i;

		location_coord.y = (y * main_stream_info_.height) / (HEATMAP_DIMS - 1) + offsets[j];
		location_coord.x = (x * main_stream_info_.width) / (HEATMAP_DIMS - 1) + offsets[j + FEATURE_SIZE];

		locations_.push_back(location_coord);
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new PoseEstimationTfStage(app);
}

static RegisterStage reg(NAME, &Create);

