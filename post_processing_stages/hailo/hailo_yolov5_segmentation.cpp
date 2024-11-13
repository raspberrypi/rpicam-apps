/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_yolo_pose.cpp - Hailo pose estimation
 */

#include <cmath>
#include <filesystem>
#include <memory>
#include <vector>

#include <hailo/hailort.hpp>

#include <libcamera/geometry.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "hailomat.hpp"
#include "instance_segmentation/yolov5seg.hpp"

#include "core/rpicam_app.hpp"
#include "hailo_postprocessing_stage.hpp"

using Size = libcamera::Size;
using PostProcFuncPtr = void (*)(HailoROIPtr, Yolov5segParams *);
using InitFuncPtr = Yolov5segParams *(*)(std::string, std::string);
using FreeFuncPtr = void (*)(void *);

namespace fs = std::filesystem;

#define NAME "hailo_yolo_segmentation"
#define POSTPROC_LIB "libyolov5seg_post.so"

namespace
{

enum overlay_status_t
{
	OVERLAY_STATUS_UNINITIALIZED = -1,
	OVERLAY_STATUS_OK,

};

const std::vector<cv::Scalar> color_table = {
	cv::Scalar(255, 0, 0),	 cv::Scalar(0, 255, 0),	  cv::Scalar(0, 0, 255),	cv::Scalar(255, 255, 0),
	cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 170, 0),	cv::Scalar(255, 0, 170),
	cv::Scalar(0, 255, 170), cv::Scalar(170, 255, 0), cv::Scalar(170, 0, 255),	cv::Scalar(0, 170, 255),
	cv::Scalar(255, 85, 0),	 cv::Scalar(85, 255, 0),  cv::Scalar(0, 255, 85),	cv::Scalar(0, 85, 255),
	cv::Scalar(85, 0, 255),	 cv::Scalar(255, 0, 85),  cv::Scalar(255, 255, 255)
};

cv::Scalar indexToColor(size_t index)
{
	return color_table[index % color_table.size()];
}

class Parallel_pixel_opencv : public cv::ParallelLoopBody
{
protected:
	cv::Vec3b *p;
	float transparency;
	int image_cols;
	int roi_cols;

public:
	Parallel_pixel_opencv(uint8_t *ptr, float transparency, int image_cols, int roi_cols)
		: p((cv::Vec3b *)ptr), transparency(transparency), image_cols(image_cols), roi_cols(roi_cols)
	{
	}
};

class ParallelPixelClassConfMask : public Parallel_pixel_opencv
{
private:
	float *mask_data;
	cv::Scalar mask_color;

public:
	ParallelPixelClassConfMask(uint8_t *ptr, uint8_t *mask_data, float transparency, int image_cols, int roi_cols,
							   cv::Scalar mask_color)
		: Parallel_pixel_opencv(ptr, transparency, image_cols, roi_cols), mask_data((float *)mask_data),
		  mask_color(mask_color)
	{
	}

	virtual void operator()(const cv::Range &r) const
	{
		constexpr float CONFIDENCE = 0.5;

		for (int i = r.start; i != r.end; ++i)
		{
			if (mask_data[i] > CONFIDENCE) // confidence is above threshold
			{
				// i the index inside the full image, convert it to the index inside the ROI
				int index = i / roi_cols * image_cols + i % roi_cols;

				p[index][0] = p[index][0] * (1 - transparency) + mask_color[0] * transparency;
				p[index][1] = p[index][1] * (1 - transparency) + mask_color[1] * transparency;
				p[index][2] = p[index][2] * (1 - transparency) + mask_color[2] * transparency;
			}
		}
	}
};

template <typename T>
void calc_destination_roi_and_resize_mask(cv::Mat &destinationROI, cv::Mat &image_planes, HailoROIPtr roi,
										  HailoMaskPtr mask, cv::Mat &resized_mask_data, T data_ptr, int cv_type)
{
	HailoBBox bbox = roi->get_bbox();
	int roi_xmin = bbox.xmin() * image_planes.cols;
	int roi_ymin = bbox.ymin() * image_planes.rows;
	int roi_width = image_planes.cols * bbox.width();
	int roi_height = image_planes.rows * bbox.height();

	// clamp the region of interest so it is inside the image planes
	roi_xmin = std::clamp(roi_xmin, 0, image_planes.cols);
	roi_ymin = std::clamp(roi_ymin, 0, image_planes.rows);
	roi_width = std::clamp(roi_width, 0, image_planes.cols - roi_xmin);
	roi_height = std::clamp(roi_height, 0, image_planes.rows - roi_ymin);

	cv::Mat mat_data = cv::Mat(mask->get_height(), mask->get_width(), cv_type, (uint8_t *)data_ptr.data());

	cv::resize(mat_data, resized_mask_data, cv::Size(roi_width, roi_height), 0, 0, cv::INTER_LINEAR);

	cv::Rect roi_rect(cv::Point(roi_xmin, roi_ymin), cv::Size(roi_width, roi_height));
	destinationROI = image_planes(roi_rect);
}

overlay_status_t draw_conf_class_mask(cv::Mat &image_planes, HailoConfClassMaskPtr mask, HailoROIPtr roi,
									  const uint mask_overlay_n_threads)
{
	cv::Mat resized_mask_data;
	cv::Mat destinationROI;

	calc_destination_roi_and_resize_mask(destinationROI, image_planes, roi, mask, resized_mask_data, mask->get_data(),
										 CV_32F);
	cv::Scalar mask_color = indexToColor(mask->get_class_id());

	if (mask_overlay_n_threads > 0)
		cv::setNumThreads(mask_overlay_n_threads);

	// perform efficient parallel matrix iteration and color every pixel its class color
	cv::parallel_for_(cv::Range(0, destinationROI.rows * destinationROI.cols),
					  ParallelPixelClassConfMask(destinationROI.data, resized_mask_data.data, mask->get_transparency(),
												 image_planes.cols, destinationROI.cols, mask_color));

	return OVERLAY_STATUS_OK;
}

overlay_status_t draw_all(cv::Mat &mat, HailoROIPtr roi, const uint mask_overlay_n_threads)
{
	overlay_status_t ret = OVERLAY_STATUS_UNINITIALIZED;
	for (auto &obj : roi->get_objects())
	{
		if (obj->get_type() == HAILO_CONF_CLASS_MASK)
		{
			HailoConfClassMaskPtr mask = std::dynamic_pointer_cast<HailoConfClassMask>(obj);
			if (mask->get_height() != 0 && mask->get_width() != 0)
				draw_conf_class_mask(mat, mask, roi, mask_overlay_n_threads);
		}
	}
	ret = OVERLAY_STATUS_OK;
	return ret;
}

} // namespace

class YoloSegmentation : public HailoPostProcessingStage
{
public:
	YoloSegmentation(RPiCamApp *app);
	~YoloSegmentation();

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	bool runInference(uint8_t *input, uint32_t *output);

	PostProcessingLib postproc_;
	Yolov5segParams *yolo_params_ = nullptr;

	// Config params
	bool show_results_;
	bool flush_results_;
	float confidence_threshold_;
};

YoloSegmentation::YoloSegmentation(RPiCamApp *app)
	: HailoPostProcessingStage(app), postproc_(PostProcLibDir(POSTPROC_LIB))
{
}

YoloSegmentation::~YoloSegmentation()
{
	if (yolo_params_)
	{
		FreeFuncPtr free_func = reinterpret_cast<FreeFuncPtr>(postproc_.GetSymbol("free_resources"));
		if (free_func)
			free_func(yolo_params_);
	}
}

char const *YoloSegmentation::Name() const
{
	return NAME;
}

void YoloSegmentation::Read(boost::property_tree::ptree const &params)
{
	show_results_ = params.get<bool>("show_results", true);
	flush_results_ = params.get<bool>("flush_results", false);
	confidence_threshold_ = params.get<float>("confidence_threshold", 0.6);

	InitFuncPtr init = reinterpret_cast<InitFuncPtr>(postproc_.GetSymbol("init"));
	const std::string config_file = params.get<std::string>("hailopp_config_file");
	if (init)
	{
		if (!fs::exists(config_file))
			throw std::runtime_error(std::string("hailo postprocess config file not found: ") + config_file);
		yolo_params_ = init(config_file, "");
	}

	HailoPostProcessingStage::Read(params);
}

void YoloSegmentation::Configure()
{
	HailoPostProcessingStage::Configure();
}

bool YoloSegmentation::Process(CompletedRequestPtr &completed_request)
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

	if (low_res_info_.pixel_format == libcamera::formats::YUV420)
	{
		StreamInfo rgb_info;
		rgb_info.width = InputTensorSize().width;
		rgb_info.height = InputTensorSize().height;
		rgb_info.stride = rgb_info.width * 3;

		input = allocator_.Allocate(rgb_info.stride * rgb_info.height);
		Yuv420ToRgb(input.get(), low_res_buffer.data(), low_res_info_, rgb_info);
	}
	else if (low_res_info_.pixel_format == libcamera::formats::RGB888 ||
			 low_res_info_.pixel_format == libcamera::formats::BGR888)
	{
		unsigned int stride = low_res_info_.width * 3;

		input = allocator_.Allocate(stride * low_res_info_.height);

		// If the stride shows we have padding on the right edge of the buffer, we must copy it out to another buffer
		// without padding.
		for (unsigned int i = 0; i < low_res_info_.height; i++)
			memcpy(input.get() + i * stride, low_res_buffer.data() + i * low_res_info_.stride, stride);
	}
	else
	{
		LOG_ERROR("Unexpected lores format " << low_res_info_.pixel_format);
		return false;
	}

	BufferWriteSync w(app_, completed_request->buffers[output_stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *output = (uint32_t *)buffer.data();

	bool success = runInference(input.get(), output);
	if (show_results_ && success)
	{
		Msg m(MsgType::Display, std::move(input), InputTensorSize(), "Segmentation");
		if (flush_results_)
			msg_queue_.Clear("Segmentation");
		msg_queue_.Post(std::move(m));
	}

	return false;
}

bool YoloSegmentation::runInference(uint8_t *input, uint32_t *output)
{
	hailort::AsyncInferJob job;
	std::vector<OutTensor> output_tensors;
	hailo_status status;

	status = HailoPostProcessingStage::DispatchJob(input, job, output_tensors);
	if (status != HAILO_SUCCESS)
		return false;

	// Prepare tensors for postprocessing.
	std::sort(output_tensors.begin(), output_tensors.end(), OutTensor::SortFunction);

	// Wait for job completion.
	status = job.wait(10s);
	if (status != HAILO_SUCCESS)
	{
		LOG_ERROR("Failed to wait for inference to finish, status = " << status);
		return false;
	}

	PostProcFuncPtr filter = reinterpret_cast<PostProcFuncPtr>(postproc_.GetSymbol("filter"));
	if (!filter)
		return false;

	HailoROIPtr roi = MakeROI(output_tensors);
	filter(roi, yolo_params_);

	cv::Mat image(InputTensorSize().height, InputTensorSize().width, CV_8UC3, (void *)input,
				  InputTensorSize().width * 3);

	std::vector<HailoDetectionPtr> detections = hailo_common::get_hailo_detections(roi);
	for (auto &detection : detections)
	{
		if (detection->get_confidence() < confidence_threshold_)
			continue;

		auto bbox = detection->get_bbox();

		cv::rectangle(image, cv::Point2f(bbox.xmin() * float(InputTensorSize().width),
										 bbox.ymin() * float(InputTensorSize().height)),
							 cv::Point2f(bbox.xmax() * float(InputTensorSize().width),
										 bbox.ymax() * float(InputTensorSize().height)),
					  cv::Scalar(0, 0, 255), 1);

		draw_all(image, detection, 0);
	}

	return true;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new YoloSegmentation(app);
}

static RegisterStage reg(NAME, &Create);
