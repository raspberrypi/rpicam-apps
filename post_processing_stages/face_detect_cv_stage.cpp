/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * face_detect_cv_stage.cpp - Face Detector implementation, using OpenCV
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

class FaceDetectCvStage : public PostProcessingStage
{
public:
	FaceDetectCvStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure();

	void Process(CompletedRequest &completed_request);

	void Stop();

private:
	void detectFeatures(cv::CascadeClassifier &cascade);
	void drawFeatures(cv::Mat &img, double scale);

	Stream *stream_;
	std::unique_ptr<std::future<void>> future_ptr_;
	std::mutex face_mutex_;
	std::mutex future_ptr_mutex_;
	Mat image_;
	std::vector<cv::Rect> faces_;
	CascadeClassifier cascade_;
	std::string cascadeName_;
	double scaling_factor_;
	int min_neighbors_;
	int min_size_;
	int max_size_;
	int refresh_rate_;
	int draw_features_;
};

#define NAME "face_detect_cv"

char const *FaceDetectCvStage::Name() const
{
	return NAME;
}

void FaceDetectCvStage::Read(boost::property_tree::ptree const &params)
{
	cascadeName_ =
		params.get<char>("cascade_name", "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml");
	if (!cascade_.load(cascadeName_))
		throw std::runtime_error("FaceDetectCvStage: failed to load haar classifier");
	scaling_factor_ = params.get<double>("scaling_factor", 1.1);
	min_neighbors_ = params.get<int>("min_neighbors", 3);
	min_size_ = params.get<int>("min_size", 32);
	max_size_ = params.get<int>("max_size", 256);
	refresh_rate_ = params.get<int>("refresh_rate", 5);
	draw_features_ = params.get<int>("draw_features", 1);
}

void FaceDetectCvStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("SobelCvStage: only YUV420 format supported");
}

void FaceDetectCvStage::Process(CompletedRequest &completed_request)
{
	int w, h, stride;
	app_->StreamDimensions(stream_, &w, &h, &stride);
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request.buffers[stream_])[0];
	uint8_t *ptr = (uint8_t *)buffer.data();
	double scale = 4;

	Mat image = Mat(h, w, CV_8U, ptr, stride);

	// Everything beyond this point is image processing...

	{
		std::unique_lock<std::mutex> lck(future_ptr_mutex_);
		if (completed_request.sequence % refresh_rate_ == 0 &&
			(!future_ptr_ || future_ptr_->wait_for(std::chrono::seconds(0)) == std::future_status::ready))
		{
			resize(image, image_, Size(), 1 / scale, 1 / scale, INTER_LINEAR);

			future_ptr_ = std::make_unique<std::future<void>>();
			*future_ptr_ = std::move(std::async(std::launch::async, [this] { detectFeatures(cascade_); }));
		}
	}

	std::unique_lock<std::mutex> lock(face_mutex_);

	std::vector<libcamera::Rectangle> temprect;
	std::transform(faces_.begin(), faces_.end(), std::back_inserter(temprect),
				   [](Rect &r) { return libcamera::Rectangle(r.x, r.y, r.width, r.height); });
	completed_request.post_process_metadata.Set("detected_faces", temprect);

	if (draw_features_ >= 1)
		drawFeatures(image, scale);
}

void FaceDetectCvStage::detectFeatures(CascadeClassifier &cascade)
{
	equalizeHist(image_, image_);

	std::vector<Rect> temp_faces;
	cascade.detectMultiScale(image_, temp_faces, scaling_factor_, min_neighbors_, CASCADE_SCALE_IMAGE,
							 Size(min_size_, min_size_), Size(max_size_, max_size_));

	std::unique_lock<std::mutex> lock(face_mutex_);
	faces_ = std::move(temp_faces);
}

void FaceDetectCvStage::drawFeatures(Mat &img, double scale)
{
	const static Scalar colors[] = {
		Scalar(255, 0, 0),	 Scalar(255, 128, 0), Scalar(255, 255, 0), Scalar(0, 255, 0),
		Scalar(0, 128, 255), Scalar(0, 255, 255), Scalar(0, 0, 255),   Scalar(255, 0, 255)
	};

	for (size_t i = 0; i < faces_.size(); i++)
	{
		Rect r = faces_[i];
		Point center;
		Scalar color = colors[i % 8];
		int radius;
		double aspect_ratio = (double)r.width / r.height;

		if (0.75 < aspect_ratio && aspect_ratio < 1.3)
		{
			center.x = cvRound((r.x + r.width * 0.5) * scale);
			center.y = cvRound((r.y + r.height * 0.5) * scale);
			radius = cvRound((r.width + r.height) * 0.25 * scale);
			circle(img, center, radius, color, 3, 8, 0);
		}
		else
			rectangle(img, Point(cvRound(r.x * scale), cvRound(r.y * scale)),
					  Point(cvRound((r.x + r.width - 1) * scale), cvRound((r.y + r.height - 1) * scale)), color, 3, 8,
					  0);
	}
}

void FaceDetectCvStage::Stop()
{
	if (future_ptr_)
		future_ptr_->wait();
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new FaceDetectCvStage(app);
}

static RegisterStage reg(NAME, &Create);
