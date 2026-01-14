/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, Kletternaut
 *
 * object_blur_stage.cpp - blur detected objects based on class names
 *
 * This stage applies blur effects to detected objects in the video stream.
 * Objects are identified by their class names (e.g., "person", "cup", "wine glass") from
 * previous detection stages like hailo_yolo_inference. Multiple blur types are supported:
 * pixelation (default), Gaussian blur, and median blur. The blur strength, bounding box
 * expansion, and target object classes can be configured via JSON.
 * This is useful for privacy applications or to obscure specific objects in real-time.
 */

#include <algorithm>
#include <string>
#include <vector>

#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include "post_processing_stages/object_detect.hpp"

#include "opencv2/imgproc.hpp"

using namespace cv;
using Stream = libcamera::Stream;

class ObjectBlurStage : public PostProcessingStage
{
public:
	ObjectBlurStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	bool shouldBlurObject(const std::string &label) const;

	Stream *stream_;
	std::vector<std::string> blur_labels_;
	std::string blur_type_ = "pixelate"; // "pixelate", "gaussian", "median"
	int blur_strength_ = 0; // 0 = auto, otherwise block size for pixelate or kernel size for blur
	int gaussian_sigma_ = 0; // 0 = auto for gaussian blur
	bool expand_box_ = false; // expand bounding box before blurring
	int expand_pixels_ = 0; // pixels to expand box by
};

#define NAME "object_blur"

char const *ObjectBlurStage::Name() const
{
	return NAME;
}

void ObjectBlurStage::Read(boost::property_tree::ptree const &params)
{
	// Read the list of object labels to blur from "overlay_blur" array
	if (params.find("overlay_blur") != params.not_found())
	{
		for (auto const &item : params.get_child("overlay_blur"))
		{
			std::string label = item.second.get_value<std::string>();
			blur_labels_.push_back(label);
			LOG(1, "ObjectBlur: Will blur objects with label: " << label);
		}
	}

	// Blur configuration
	blur_type_ = params.get<std::string>("blur_type", "pixelate");
	blur_strength_ = params.get<int>("blur_strength", 0);
	gaussian_sigma_ = params.get<int>("gaussian_sigma", 0);
	expand_box_ = params.get<bool>("expand_box", false);
	expand_pixels_ = params.get<int>("expand_pixels", 0);
	
	LOG(1, "ObjectBlur: blur_type=" << blur_type_ << ", blur_strength=" << blur_strength_ 
	       << ", expand_box=" << expand_box_ << ", expand_pixels=" << expand_pixels_);
	
	if (blur_labels_.empty())
	{
		LOG(1, "ObjectBlur: Warning - no labels specified in 'overlay_blur'");
	}
}

void ObjectBlurStage::Configure()
{
	stream_ = app_->GetMainStream();
}

bool ObjectBlurStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_ || blur_labels_.empty())
		return false;

	std::vector<Detection> detections;
	completed_request->post_process_metadata.Get("object_detect.results", detections);

	if (detections.empty())
		return false;

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	StreamInfo info = app_->GetStreamInfo(stream_);

	Mat image(info.height, info.width, CV_8U, (uint32_t *)buffer.data(), info.stride);

	int blurred_count = 0;
	for (auto &detection : detections)
	{
		if (shouldBlurObject(detection.name))
		{
			Rect roi(detection.box.x, detection.box.y, detection.box.width, detection.box.height);
			
			// Optional: expand bounding box
			if (expand_box_ || expand_pixels_ > 0)
			{
				int expand = expand_pixels_ > 0 ? expand_pixels_ : std::max(10, (int)(roi.width * 0.1));
				roi.x -= expand;
				roi.y -= expand;
				roi.width += 2 * expand;
				roi.height += 2 * expand;
			}
			
			// Clamp to image bounds
			roi.x = std::max(0, roi.x);
			roi.y = std::max(0, roi.y);
			roi.width = std::min(roi.width, (int)info.width - roi.x);
			roi.height = std::min(roi.height, (int)info.height - roi.y);

			if (roi.width > 0 && roi.height > 0)
			{
				Mat region = image(roi);
				
				if (blur_type_ == "gaussian")
				{
					// Gaussian blur
					int kernel_size = blur_strength_;
					if (kernel_size == 0)
						kernel_size = std::max(5, std::min(51, (int)(roi.width / 10)));
					
					// Kernel size must be odd
					if (kernel_size % 2 == 0)
						kernel_size++;
					
					double sigma = gaussian_sigma_;
					if (sigma == 0)
						sigma = kernel_size / 6.0;
					
					GaussianBlur(region, region, Size(kernel_size, kernel_size), sigma);
				}
				else if (blur_type_ == "median")
				{
					// Median blur
					int kernel_size = blur_strength_;
					if (kernel_size == 0)
						kernel_size = std::max(5, std::min(51, (int)(roi.width / 10)));
					
					// Kernel size must be odd
					if (kernel_size % 2 == 0)
						kernel_size++;
					
					medianBlur(region, region, kernel_size);
				}
				else // pixelate (default)
				{
					// Pixelation effect
					int block_size = blur_strength_;
					if (block_size == 0)
					{
						// Auto-calculate based on bounding box size
						block_size = std::max(8, std::min(32, (int)(roi.width / 15)));
					}
					
					int small_width = std::max(1, roi.width / block_size);
					int small_height = std::max(1, roi.height / block_size);
					
					Mat small;
					resize(region, small, Size(small_width, small_height), 0, 0, INTER_LINEAR);
					resize(small, region, Size(roi.width, roi.height), 0, 0, INTER_NEAREST);
				}
				
				blurred_count++;
			}
		}
	}

	if (blurred_count > 0)
		LOG(2, "ObjectBlur: Blurred " << blurred_count << " objects");

	return false;
}

bool ObjectBlurStage::shouldBlurObject(const std::string &label) const
{
	return std::find(blur_labels_.begin(), blur_labels_.end(), label) != blur_labels_.end();
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectBlurStage(app);
}

static RegisterStage reg(NAME, &Create);
