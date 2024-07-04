/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_yolo_pose.cpp - Hailo pose estimation
 */

#include <cmath>
#include <vector>

#include <hailo/hailort.hpp>

#include <libcamera/geometry.h>
#include <opencv2/imgproc.hpp>

#include "pose_estimation/yolov8pose_postprocess.hpp"

#include "core/rpicam_app.hpp"
#include "hailo_postprocessing_stage.hpp"

using Size = libcamera::Size;
using PostProcFuncPtr = std::pair<std::vector<KeyPt>, std::vector<PairPairs>>(*)(HailoROIPtr);

#define NAME "hailo_yolo_pose"
#define POSTPROC_LIB "libyolov8pose_post.so"

namespace
{

constexpr Size INPUT_TENSOR_SIZE { 640, 640 };

} // namespace

class YoloPose : public HailoPostProcessingStage
{
public:
	YoloPose(RPiCamApp *app);

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	void runInference(const uint8_t *input, uint32_t *output);

	PostProcessingLib postproc_;
};

YoloPose::YoloPose(RPiCamApp *app)
	: HailoPostProcessingStage(app), postproc_(PostProcLibDir(POSTPROC_LIB))
{
}

char const *YoloPose::Name() const
{
	return NAME;
}

void YoloPose::Read(boost::property_tree::ptree const &params)
{
	HailoPostProcessingStage::Read(params);
}

void YoloPose::Configure()
{
	HailoPostProcessingStage::Configure();
}

bool YoloPose::Process(CompletedRequestPtr &completed_request)
{
	if (!HailoPostProcessingStage::Ready())
	{
		LOG_ERROR("HailoRT not ready!");
		return false;
	}

	if (low_res_info_.width != INPUT_TENSOR_SIZE.width || low_res_info_.height != INPUT_TENSOR_SIZE.height)
	{
		LOG_ERROR("Wrong low res size, expecting " << INPUT_TENSOR_SIZE.toString());
		return false;
	}

	std::shared_ptr<uint8_t> input;
	if (low_res_info_.pixel_format != libcamera::formats::BGR888)
	{
		StreamInfo rgb_info;
		rgb_info.width = INPUT_TENSOR_SIZE.width;
		rgb_info.height = INPUT_TENSOR_SIZE.height;
		rgb_info.stride = rgb_info.width * 3;

		BufferReadSync r(app_, completed_request->buffers[low_res_stream_]);
		libcamera::Span<uint8_t> buffer = r.Get()[0];
		input = allocator_.Allocate(rgb_info.stride * rgb_info.height);
		Yuv420ToRgb(input.get(), buffer.data(), low_res_info_, rgb_info);
	}

	BufferWriteSync w(app_, completed_request->buffers[output_stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *output = (uint32_t *)buffer.data();

	runInference(input.get(), output);

	return false;
}

void YoloPose::runInference(const uint8_t *input, uint32_t *output)
{
	hailort::AsyncInferJob job;
	std::vector<OutTensor> output_tensors;
	hailo_status status;

	status = HailoPostProcessingStage::DispatchJob(input, job, output_tensors);
	if (status != HAILO_SUCCESS)
		return;

	// Prepare tensors for postprocessing.
	std::sort(output_tensors.begin(), output_tensors.end(), OutTensor::SortFunction);

	// Wait for job completion.
	status = job.wait(1s);
	if (status != HAILO_SUCCESS)
	{
		LOG_ERROR("Failed to wait for inference to finish, status = " << status);
		return;
	}

	PostProcFuncPtr filter = reinterpret_cast<PostProcFuncPtr>(postproc_.GetSymbol("yolov8"));
	if (!filter)
		return;

	HailoROIPtr roi = MakeROI(output_tensors);
	std::pair<std::vector<KeyPt>, std::vector<PairPairs>> keypoints_and_pairs = filter(roi);

	std::vector<HailoDetectionPtr> detections = hailo_common::get_hailo_detections(roi);
	cv::Mat image(output_stream_info_.height, output_stream_info_.width, CV_8U, output, output_stream_info_.stride);

	for (auto &detection : detections)
	{
		if (detection->get_confidence() == 0)
			continue;

		HailoBBox bbox = detection->get_bbox();

		cv::rectangle(image,
					  cv::Point2f(bbox.xmin() * float(output_stream_info_.width),
								  bbox.ymin() * float(output_stream_info_.height)),
					  cv::Point2f(bbox.xmax() * float(output_stream_info_.width),
								  bbox.ymax() * float(output_stream_info_.height)),
					  cv::Scalar(0, 0, 255), 1);
	}

	for (auto &keypoint : keypoints_and_pairs.first)
	{
		cv::circle(image,
				   cv::Point(keypoint.xs * float(output_stream_info_.width),
							 keypoint.ys * float(output_stream_info_.height)),
				   3, cv::Scalar(255, 0, 255), -1);
	}

	for (PairPairs &p : keypoints_and_pairs.second)
	{
		auto pt1 =
			cv::Point(p.pt1.first * float(output_stream_info_.width), p.pt1.second * float(output_stream_info_.height));
		auto pt2 =
			cv::Point(p.pt2.first * float(output_stream_info_.width), p.pt2.second * float(output_stream_info_.height));
		cv::line(image, pt1, pt2, cv::Scalar(255, 0, 255), 3);
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new YoloPose(app);
}

static RegisterStage reg(NAME, &Create);
