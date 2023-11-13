/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * negate_stage.cpp - image negate effect
 */

#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <memory>
#include <vector>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/imgproc.hpp"

using namespace cv;

using Stream = libcamera::Stream;

enum Features
{
	nose,
	leftEye,
	rightEye,
	leftEar,
	rightEar,
	leftShoulder,
	rightShoulder,
	leftElbow,
	rightElbow,
	leftWrist,
	rightWrist,
	leftHip,
	rightHip,
	leftKnee,
	rightKnee,
	leftAnkle,
	rightAnkle
};

constexpr int FEATURE_SIZE = 17;

class PlotPoseCvStage : public PostProcessingStage
{
public:
	PlotPoseCvStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	void drawFeatures(cv::Mat &img, std::vector<Point> locations, std::vector<float> confidences);

	Stream *stream_;
	float confidence_threshold_;
};

#define NAME "plot_pose_cv"

char const *PlotPoseCvStage::Name() const
{
	return NAME;
}

void PlotPoseCvStage::Configure()
{
	stream_ = app_->GetMainStream();
}

void PlotPoseCvStage::Read(boost::property_tree::ptree const &params)
{
	confidence_threshold_ = params.get<float>("confidence_threshold", -1.0);
}

bool PlotPoseCvStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *ptr = (uint32_t *)buffer.data();
	StreamInfo info = app_->GetStreamInfo(stream_);

	std::vector<cv::Rect> rects;
	std::vector<libcamera::Point> lib_locations;
	std::vector<Point> cv_locations;
	std::vector<float> confidences;

	completed_request->post_process_metadata.Get("pose_estimation.locations", lib_locations);
	completed_request->post_process_metadata.Get("pose_estimation.confidences", confidences);

	if (!confidences.empty() && !lib_locations.empty())
	{
		Mat image(info.height, info.width, CV_8U, ptr, info.stride);
		for (libcamera::Point lib_location : lib_locations)
		{
			Point cv_location;
			cv_location.x = lib_location.x;
			cv_location.y = lib_location.y;
			cv_locations.push_back(cv_location);
		}
		drawFeatures(image, cv_locations, confidences);
	}
	return false;
}

void PlotPoseCvStage::drawFeatures(Mat &img, std::vector<cv::Point> locations, std::vector<float> confidences)
{
	Scalar colour = Scalar(255, 255, 255);
	int radius = 5;

	for (int i = 0; i < FEATURE_SIZE; i++)
	{
		if (confidences[i] < confidence_threshold_)
			circle(img, locations[i], radius, colour, 2, 8, 0);
	}

	if (confidences[leftShoulder] > confidence_threshold_)
	{
		if (confidences[rightShoulder] > confidence_threshold_)
			line(img, locations[leftShoulder], locations[rightShoulder], colour, 2);

		if (confidences[leftElbow] > confidence_threshold_)
			line(img, locations[leftShoulder], locations[leftElbow], colour, 2);

		if (confidences[leftHip] > confidence_threshold_)
			line(img, locations[leftShoulder], locations[leftHip], colour, 2);
	}
	if (confidences[rightShoulder] > confidence_threshold_)
	{
		if (confidences[rightElbow] > confidence_threshold_)
			line(img, locations[rightShoulder], locations[rightElbow], colour, 2);

		if (confidences[rightHip] > confidence_threshold_)
			line(img, locations[rightShoulder], locations[rightHip], colour, 2);
	}
	if (confidences[leftElbow] > confidence_threshold_)
	{
		if (confidences[leftWrist] > confidence_threshold_)
			line(img, locations[leftElbow], locations[leftWrist], colour, 2);
	}
	if (confidences[rightElbow] > confidence_threshold_)
	{
		if (confidences[rightWrist] > confidence_threshold_)
			line(img, locations[rightElbow], locations[rightWrist], colour, 2);
	}
	if (confidences[leftHip] > confidence_threshold_)
	{
		if (confidences[rightHip] > confidence_threshold_)
			line(img, locations[leftHip], locations[rightHip], colour, 2);

		if (confidences[leftKnee] > confidence_threshold_)
			line(img, locations[leftHip], locations[leftKnee], colour, 2);
	}
	if (confidences[leftKnee] > confidence_threshold_)
	{
		if (confidences[leftAnkle] > confidence_threshold_)
			line(img, locations[leftKnee], locations[leftAnkle], colour, 2);
	}
	if (confidences[rightKnee] > confidence_threshold_)
	{
		if (confidences[rightHip] > confidence_threshold_)
			line(img, locations[rightKnee], locations[rightHip], colour, 2);

		if (confidences[rightAnkle] > confidence_threshold_)
			line(img, locations[rightKnee], locations[rightAnkle], colour, 2);
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new PlotPoseCvStage(app);
}

static RegisterStage reg(NAME, &Create);
