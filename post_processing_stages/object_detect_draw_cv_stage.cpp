/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_detect_draw_cv_stage.cpp - draw object detection results
 */

#include "opencv2/imgproc.hpp"

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "object_detect.hpp"

using namespace cv;

using Rectange = libcamera::Rectangle;
using Stream = libcamera::Stream;

class ObjectDetectDrawCvStage : public PostProcessingStage
{
public:
	ObjectDetectDrawCvStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	int line_thickness_;
	double font_size_;
};

#define NAME "object_detect_draw_cv"

char const *ObjectDetectDrawCvStage::Name() const
{
	return NAME;
}

void ObjectDetectDrawCvStage::Configure()
{
	// Only draw on image if a low res stream was specified.
	stream_ = app_->LoresStream() ? app_->GetMainStream() : nullptr;
}

void ObjectDetectDrawCvStage::Read(boost::property_tree::ptree const &params)
{
	line_thickness_ = params.get<int>("line_thickness", 1);
	font_size_ = params.get<double>("font_size", 1.0);
}

bool ObjectDetectDrawCvStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *ptr = (uint32_t *)buffer.data();
	StreamInfo info = app_->GetStreamInfo(stream_);

	std::vector<Detection> detections;

	completed_request->post_process_metadata.Get("object_detect.results", detections);

	Mat image(info.height, info.width, CV_8U, ptr, info.stride);
	Scalar colour = Scalar(255, 255, 255);
	int font = FONT_HERSHEY_SIMPLEX;

	for (auto &detection : detections)
	{
		Rect r(detection.box.x, detection.box.y, detection.box.width, detection.box.height);
		rectangle(image, r, colour, line_thickness_);
		std::stringstream text_stream;
		text_stream << detection.name << " " << (int)(detection.confidence * 100) << "%";
		std::string text = text_stream.str();
		int baseline = 0;
		Size size = getTextSize(text, font, font_size_, 2, &baseline);
		Point text_origin(detection.box.x + 5, detection.box.y + size.height + 5);
		putText(image, text, text_origin, font, font_size_, colour, 2);
	}

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectDetectDrawCvStage(app);
}

static RegisterStage reg(NAME, &Create);
