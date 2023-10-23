/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * motion_detect_stage.cpp - motion detector
 */

// A simple motion detector. It needs to be given a low resolution image and it
// compares pixels in the current low res image against the value in the corresponding
// location in the previous one. If it exceeds a threshold it gets counted as
// "different". If enough pixels are different, that indicates "motion".
// A low res image of something like 128x96 is probably more than enough, and you
// can always subsample with hskip and vksip.

// Because this gets run in parallel by the post-processing framework, it means
// the "previous frame" is not totally guaranteed to be the actual previous one,
// though in practice it is, and it doesn't actually matter even if it wasn't.

// The stage adds "motion_detect.result" to the metadata. When this claims motion,
// the application can take that as true immediately. To be sure there's no motion,
// an application should probably wait for "a few frames" of "no motion".

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

using Stream = libcamera::Stream;

class MotionDetectStage : public PostProcessingStage
{
public:
	MotionDetectStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	// In the Config, dimensions are given as fractions of the lores image size.
	struct Config
	{
		float roi_x, roi_y;
		float roi_width, roi_height;
		int hskip, vskip;
		float difference_m;
		int difference_c;
		float region_threshold;
		int frame_period;
		bool verbose;
	} config_;
	Stream *stream_;
	unsigned lores_stride_;
	// Here we convert the dimensions to pixel locations in the lores image, as if subsampled
	// by hskip and vskip.
	unsigned int roi_x_, roi_y_;
	unsigned int roi_width_, roi_height_;
	unsigned int region_threshold_;
	std::vector<uint8_t> previous_frame_;
	bool first_time_;
	bool motion_detected_;
	std::mutex mutex_;
};

#define NAME "motion_detect"

char const *MotionDetectStage::Name() const
{
	return NAME;
}

void MotionDetectStage::Read(boost::property_tree::ptree const &params)
{
	config_.roi_x = params.get<float>("roi_x", 0.0);
	config_.roi_y = params.get<float>("roi_y", 0.0);
	config_.roi_width = params.get<float>("roi_width", 1.0);
	config_.roi_height = params.get<float>("roi_height", 1.0);
	config_.hskip = params.get<int>("hskip", 1);
	config_.vskip = params.get<int>("vskip", 1);
	config_.difference_m = params.get<float>("difference_m", 0.1);
	config_.difference_c = params.get<int>("difference_c", 10);
	config_.region_threshold = params.get<float>("region_threshold", 0.005);
	config_.frame_period = params.get<int>("frame_period", 5);
	config_.verbose = params.get<int>("verbose", 0);
}

void MotionDetectStage::Configure()
{
	StreamInfo info;
	stream_ = app_->LoresStream(&info);
	if (!stream_)
		return;

	config_.hskip = std::max(config_.hskip, 1);
	config_.vskip = std::max(config_.vskip, 1);
	info.width /= config_.hskip;
	info.height /= config_.vskip;
	lores_stride_ = info.stride * config_.vskip;

	// Turn fractions of the lores image into actual pixel numbers. Store them as if in
	// an image subsampled by hskip and vskip.
	roi_x_ = config_.roi_x * info.width;
	roi_y_ = config_.roi_y * info.height;
	roi_width_ = config_.roi_width * info.width;
	roi_height_ = config_.roi_height * info.height;
	region_threshold_ = config_.region_threshold * roi_width_ * roi_height_;

	roi_x_ = std::clamp(roi_x_, 0u, info.width);
	roi_y_ = std::clamp(roi_y_, 0u, info.height);
	roi_width_ = std::clamp(roi_width_, 0u, info.width - roi_x_);
	roi_height_ = std::clamp(roi_height_, 0u, info.height - roi_y_);
	region_threshold_ = std::clamp(region_threshold_, 0u, roi_width_ * roi_height_);

	if (config_.verbose)
		LOG(1, "Lores: " << info.width << "x" << info.height << " roi: (" << roi_x_ << "," << roi_y_ << ") "
						 << roi_width_ << "x" << roi_height_ << " threshold: " << region_threshold_);

	previous_frame_.resize(roi_width_ * roi_height_);
	first_time_ = true;
	motion_detected_ = false;
}

bool MotionDetectStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	if (config_.frame_period && completed_request->sequence % config_.frame_period)
		return false;

	BufferReadSync r(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = r.Get()[0];
	uint8_t *image = buffer.data();

	// We need to protect access to first_time_, previous_frame_ and motion_detected_.
	std::lock_guard<std::mutex> lock(mutex_);

	if (first_time_)
	{
		first_time_ = false;
		for (unsigned int y = 0; y < roi_height_; y++)
		{
			uint8_t *new_value_ptr = image + (roi_y_ + y) * lores_stride_ + roi_x_ * config_.hskip;
			uint8_t *old_value_ptr = &previous_frame_[0] + y * roi_width_;
			for (unsigned int x = 0; x < roi_width_; x++, new_value_ptr += config_.hskip)
				*(old_value_ptr++) = *new_value_ptr;
		}

		completed_request->post_process_metadata.Set("motion_detect.result", motion_detected_);

		return false;
	}

	bool motion_detected = false;
	unsigned int regions = 0;

	// Count the lores pixels where the difference between the new and previous values
	// exceeds the threshold. At the same time, update the previous image buffer.
	for (unsigned int y = 0; y < roi_height_; y++)
	{
		uint8_t *new_value_ptr = image + (roi_y_ + y) * lores_stride_ + roi_x_ * config_.hskip;
		uint8_t *old_value_ptr = &previous_frame_[0] + y * roi_width_;
		for (unsigned int x = 0; x < roi_width_; x++, new_value_ptr += config_.hskip)
		{
			int new_value = *new_value_ptr;
			int old_value = *old_value_ptr;
			*(old_value_ptr++) = new_value;
			regions += std::abs(new_value - old_value) > config_.difference_m * old_value + config_.difference_c;
			motion_detected = regions >= region_threshold_;
		}
	}

	if (config_.verbose && motion_detected != motion_detected_)
		LOG(1, "Motion " << (motion_detected ? "detected" : "stopped"));

	motion_detected_ = motion_detected;
	completed_request->post_process_metadata.Set("motion_detect.result", motion_detected);

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new MotionDetectStage(app);
}

static RegisterStage reg(NAME, &Create);
