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
#include <libcamera/stream.h>
#include <memory>
#include <vector>

#include "../core/libcamera_app.hpp"
#include "../core/post_processing_stage.hpp"
#include "libcamera/geometry.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"

using namespace cv;

using Stream = libcamera::Stream;

class PlotPoseCvStage : public PostProcessingStage
{
public:
	PlotPoseCvStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params);

	void Configure();

	bool Process(CompletedRequest &completed_request);

private:
	void drawFeatures(cv::Mat &img, std::vector<Point> locations, std::vector<float> confidences);

	Stream *stream_;
	std::mutex future_ptr_mutex_;
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
	confidence_threshold_ = params.get<float>("confidence_threshold_", -1.0);
}

bool PlotPoseCvStage::Process(CompletedRequest &completed_request)
{
	std::unique_lock<std::mutex> lck(future_ptr_mutex_);
	int w, h, stride;
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request.buffers[stream_])[0];
	uint32_t *ptr = (uint32_t *)buffer.data();
	app_->StreamDimensions(stream_, &w, &h, &stride);

	std::vector<cv::Rect> rects;
	std::vector<libcamera::Point> lib_locations;
	std::vector<Point> cv_locations;
	std::vector<float> confidences;

	completed_request.post_process_metadata.Get("locations", lib_locations);
	completed_request.post_process_metadata.Get("confidences", confidences);

	if (!confidences.empty() && !lib_locations.empty())
	{
		Mat image(h, w, CV_8U, ptr, stride);
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
	Scalar color = Scalar(255, 255, 255);
	int radius = 5;

	for (int i = 0; i < 17; i++)
	{
		if (confidences[i] < confidence_threshold_)
			circle(img, locations[i], radius, color, 3, 8, 0);
	}

	if (confidences[5] > confidence_threshold_)
	{
		if (confidences[6] > confidence_threshold_)
			line(img, locations[5], locations[6], color, 3);

		if (confidences[7] > confidence_threshold_)
			line(img, locations[5], locations[7], color, 3);

		if (confidences[11] > confidence_threshold_)
			line(img, locations[5], locations[11], color, 3);
	}
	if (confidences[6] > confidence_threshold_)
	{
		if (confidences[8] > confidence_threshold_)
			line(img, locations[6], locations[8], color, 3);

		if (confidences[12] > confidence_threshold_)
			line(img, locations[6], locations[12], color, 3);
	}
	if (confidences[7] > confidence_threshold_)
	{
		if (confidences[9] > confidence_threshold_)
			line(img, locations[7], locations[9], color, 3);
	}
	if (confidences[8] > confidence_threshold_)
	{
		if (confidences[10] > confidence_threshold_)
			line(img, locations[8], locations[10], color, 3);
	}
	if (confidences[11] > confidence_threshold_)
	{
		if (confidences[12] > confidence_threshold_)
			line(img, locations[11], locations[12], color, 3);

		if (confidences[13] > confidence_threshold_)
			line(img, locations[11], locations[13], color, 3);
	}
	if (confidences[13] > confidence_threshold_)
	{
		if (confidences[15] > confidence_threshold_)
			line(img, locations[13], locations[15], color, 3);
	}
	if (confidences[14] > confidence_threshold_)
	{
		if (confidences[12] > confidence_threshold_)
			line(img, locations[14], locations[12], color, 3);

		if (confidences[16] > confidence_threshold_)
			line(img, locations[14], locations[16], color, 3);
	}
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new PlotPoseCvStage(app);
}

static RegisterStage reg(NAME, &Create);
