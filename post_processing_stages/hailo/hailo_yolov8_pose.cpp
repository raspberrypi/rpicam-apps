/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_yolo_pose.cpp - Hailo pose estimation
 */

#include <cmath>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <hailo/hailort.hpp>

#include <libcamera/geometry.h>
#include <opencv2/imgproc.hpp>

#include "pose_estimation/yolov8pose_postprocess.hpp"

#include "core/rpicam_app.hpp"
#include "hailo_postprocessing_stage.hpp"

using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;
using PostProcFuncPtr = std::pair<std::vector<KeyPt>, std::vector<PairPairs>>(*)(HailoROIPtr);

#define NAME "hailo_yolo_pose"
#define POSTPROC_LIB "libyolov8pose_post.so"

class YoloPose : public HailoPostProcessingStage
{
public:
	YoloPose(RPiCamApp *app);

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	void runInference(const uint8_t *input, uint32_t *output, const std::vector<Rectangle> &scaler_crops);

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

	if (low_res_info_.width != InputTensorSize().width || low_res_info_.height != InputTensorSize().height)
	{
		LOG_ERROR("Wrong low res size, expecting " << InputTensorSize().toString());
		return false;
	}

	BufferReadSync r(app_, completed_request->buffers[low_res_stream_]);
	libcamera::Span<uint8_t> low_res_buffer = r.Get()[0];
	std::shared_ptr<uint8_t> input;
	uint8_t *input_ptr;

	if (low_res_info_.pixel_format == libcamera::formats::YUV420)
	{
		StreamInfo rgb_info;
		rgb_info.width = InputTensorSize().width;
		rgb_info.height = InputTensorSize().height;
		rgb_info.stride = rgb_info.width * 3;

		input = allocator_.Allocate(rgb_info.stride * rgb_info.height);
		input_ptr = input.get();

		Yuv420ToRgb(input.get(), low_res_buffer.data(), low_res_info_, rgb_info);
	}
	else if (low_res_info_.pixel_format == libcamera::formats::RGB888 ||
			 low_res_info_.pixel_format == libcamera::formats::BGR888)
	{
		unsigned int stride = low_res_info_.width * 3;

		input = allocator_.Allocate(stride * low_res_info_.height);
		input_ptr = input.get();

		// If the stride shows we have padding on the right edge of the buffer, we must copy it out to another buffer
		// without padding.
		for (unsigned int i = 0; i < low_res_info_.height; i++)
			memcpy(input_ptr + i * stride, low_res_buffer.data() + i * low_res_info_.stride, stride);
	}
	else
	{
		LOG_ERROR("Unexpected lores format " << low_res_info_.pixel_format);
		return false;
	}

	std::vector<Rectangle> scaler_crops;
	auto scaler_crop = completed_request->metadata.get(controls::ScalerCrop);
	auto rpi_scaler_crop = completed_request->metadata.get(controls::rpi::ScalerCrops);

	if (rpi_scaler_crop)
	{
		for (unsigned int i = 0; i < rpi_scaler_crop->size(); i++)
			scaler_crops.push_back(rpi_scaler_crop->data()[i]);
	}
	else if (scaler_crop)
	{
		// Push-back twice, once for main, once for low res.
		scaler_crops.push_back(*scaler_crop);
		scaler_crops.push_back(*scaler_crop);
	}

	BufferWriteSync w(app_, completed_request->buffers[output_stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *output = (uint32_t *)buffer.data();

	runInference(input_ptr, output, scaler_crops);

	{
		Msg m(MsgType::Display, std::move(input), InputTensorSize(), "Pose");
		msg_queue_.Clear("Pose");
		msg_queue_.Post(std::move(m));
	}

	return false;
}

void YoloPose::runInference(const uint8_t *input, uint32_t *output, const std::vector<Rectangle> &scaler_crops)
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
	cv::Mat image(InputTensorSize().height, InputTensorSize().width, CV_8UC3, (void *)input,
				  InputTensorSize().width * 3);

	for (auto &detection : detections)
	{
		if (detection->get_confidence() == 0)
			continue;

		HailoBBox bbox = detection->get_bbox();
		const float x0 = std::max(bbox.xmin(), 0.0f);
		const float x1 = std::min(bbox.xmax(), 1.0f);
		const float y0 = std::max(bbox.ymin(), 0.0f);
		const float y1 = std::min(bbox.ymax(), 1.0f);
		Rectangle r = ConvertInferenceCoordinates({ x0, y0, x1 - x0, y1 - y0 }, scaler_crops);
		cv::rectangle(image, cv::Point2f(r.x, r.y), cv::Point2f(r.x + r.width, r.y + r.height), cv::Scalar(0, 0, 255),
					  1);
	}

	for (auto &keypoint : keypoints_and_pairs.first)
	{
		cv::circle(image, cv::Point(keypoint.xs * InputTensorSize().width, keypoint.ys * InputTensorSize().height), 3,
				   cv::Scalar(255, 0, 255), -1);
	}

	for (const PairPairs &p : keypoints_and_pairs.second)
	{
		cv::line(image, cv::Point(p.pt1.first * InputTensorSize().width, p.pt1.second * InputTensorSize().height),
				 cv::Point(p.pt2.first * InputTensorSize().width, p.pt2.second * InputTensorSize().height),
				 cv::Scalar(255, 0, 255), 3);
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new YoloPose(app);
}

static RegisterStage reg(NAME, &Create);
