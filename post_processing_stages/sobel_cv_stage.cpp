/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * sobel_cv_stage.cpp - Sobel filter implementation, using OpenCV
 */

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;

using Stream = libcamera::Stream;

class SobelCvStage : public PostProcessingStage
{
public:
	SobelCvStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	int ksize_ = 3;
};

#define NAME "sobel_cv"

char const *SobelCvStage::Name() const
{
	return NAME;
}

void SobelCvStage::Read(boost::property_tree::ptree const &params)
{
	ksize_ = params.get<int16_t>("ksize", 3);
}

void SobelCvStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("SobelCvStage: only YUV420 format supported");
}

bool SobelCvStage::Process(CompletedRequestPtr &completed_request)
{
	StreamInfo info = app_->GetStreamInfo(stream_);
	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint8_t *ptr = (uint8_t *)buffer.data();

	//Everything beyond this point is image processing...

	uint8_t value = 128;
	int num = (info.stride * info.height) / 2;
	Mat src = Mat(info.height, info.width, CV_8U, ptr, info.stride);
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	memset(ptr + info.stride * info.height, value, num);

	// Remove noise by blurring with a Gaussian filter ( kernal size = 3 )
	GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);

	Mat grad_x, grad_y;

	//Scharr(src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
	Sobel(src, grad_x, ddepth, 1, 0, ksize_, scale, delta, BORDER_DEFAULT);
	//Scharr(src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);
	Sobel(src, grad_y, ddepth, 0, 1, ksize_, scale, delta, BORDER_DEFAULT);

	// converting back to CV_8U
	convertScaleAbs(grad_x, grad_x);
	convertScaleAbs(grad_y, grad_y);

	//weight the x and y gradients and add their magnitudes
	addWeighted(grad_x, 0.5, grad_y, 0.5, 0, src);

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new SobelCvStage(app);
}

static RegisterStage reg(NAME, &Create);
