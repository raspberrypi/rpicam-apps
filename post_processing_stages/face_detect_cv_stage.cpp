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

#include <libcamera/geometry.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"

using namespace cv;

using Stream = libcamera::Stream;

class FaceDetectCvStage : public PostProcessingStage
{
public:
	FaceDetectCvStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	void Stop() override;

private:
	void detectFeatures(cv::CascadeClassifier &cascade);
	void drawFeatures(cv::Mat &img);

	Stream *stream_;
	StreamInfo low_res_info_;
	Stream *full_stream_;
	StreamInfo full_stream_info_;
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
	stream_ = nullptr;
	full_stream_ = nullptr;

	if (app_->StillStream()) // for stills capture, do nothing
		return;

	// Otherwise we expect there to be a lo res stream that we will use.
	stream_ = app_->LoresStream();
	if (!stream_)
		throw std::runtime_error("FaceDetectCvStage: no low resolution stream");
	// (the lo res stream can only be YUV420)
	low_res_info_ = app_->GetStreamInfo(stream_);

	// We also expect there to be a "full resolution" stream which defines the output coordinate
	// system, and we can optionally draw the faces there too.
	full_stream_ = app_->GetMainStream();
	if (!full_stream_)
		throw std::runtime_error("FaceDetectCvStage: no full resolution stream available");
	full_stream_info_ = app_->GetStreamInfo(full_stream_);
	if (draw_features_ && full_stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("FaceDetectCvStage: drawing only supported for YUV420 images");
}

bool FaceDetectCvStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	{
		std::unique_lock<std::mutex> lck(future_ptr_mutex_);
		if (completed_request->sequence % refresh_rate_ == 0 &&
			(!future_ptr_ || future_ptr_->wait_for(std::chrono::seconds(0)) == std::future_status::ready))
		{
			BufferReadSync r(app_, completed_request->buffers[stream_]);
			libcamera::Span<uint8_t> buffer = r.Get()[0];
			uint8_t *ptr = (uint8_t *)buffer.data();
			Mat image(low_res_info_.height, low_res_info_.width, CV_8U, ptr, low_res_info_.stride);
			image_ = image.clone();

			future_ptr_ = std::make_unique<std::future<void>>();
			*future_ptr_ = std::async(std::launch::async, [this] { detectFeatures(cascade_); });
		}
	}

	std::unique_lock<std::mutex> lock(face_mutex_);

	std::vector<libcamera::Rectangle> temprect;
	std::transform(faces_.begin(), faces_.end(), std::back_inserter(temprect),
				   [](Rect &r) { return libcamera::Rectangle(r.x, r.y, r.width, r.height); });
	completed_request->post_process_metadata.Set("detected_faces", temprect);

	if (draw_features_)
	{
		BufferWriteSync w(app_, completed_request->buffers[full_stream_]);
		libcamera::Span<uint8_t> buffer = w.Get()[0];
		uint8_t *ptr = (uint8_t *)buffer.data();
		Mat image(full_stream_info_.height, full_stream_info_.width, CV_8U, ptr, full_stream_info_.stride);
		drawFeatures(image);
	}

	return false;
}

void FaceDetectCvStage::detectFeatures(CascadeClassifier &cascade)
{
	equalizeHist(image_, image_);

	std::vector<Rect> temp_faces;
	cascade.detectMultiScale(image_, temp_faces, scaling_factor_, min_neighbors_, CASCADE_SCALE_IMAGE,
							 Size(min_size_, min_size_), Size(max_size_, max_size_));

	// Scale faces back to the size and location in the full res image.
	double scale_x = full_stream_info_.width / (double)low_res_info_.width;
	double scale_y = full_stream_info_.height / (double)low_res_info_.height;
	for (auto &face : temp_faces)
	{
		face.x *= scale_x;
		face.y *= scale_y;
		face.width *= scale_x;
		face.height *= scale_y;
	}
	std::unique_lock<std::mutex> lock(face_mutex_);
	faces_ = std::move(temp_faces);
}

void FaceDetectCvStage::drawFeatures(Mat &img)
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
			center.x = cvRound(r.x + r.width * 0.5);
			center.y = cvRound(r.y + r.height * 0.5);
			radius = cvRound((r.width + r.height) * 0.25);
			circle(img, center, radius, color, 3, 8, 0);
		}
		else
			rectangle(img, Point(cvRound(r.x), cvRound(r.y)),
					  Point(cvRound(r.x + r.width - 1), cvRound(r.y + r.height - 1)), color, 3, 8, 0);
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
