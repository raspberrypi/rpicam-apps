/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_retinaface.cpp - Hailo facial keypoints
 */

#include <cmath>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <hailo/hailort.hpp>

#include <libcamera/geometry.h>
#include <opencv2/imgproc.hpp>

#include "common/math.hpp"
#include "common/nms.hpp"
#include "common/tensors.hpp"

#include "detection/scrfd.hpp"

#include "hailo_xtensor.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xpad.hpp"
#include "xtensor/xview.hpp"

#include "core/rpicam_app.hpp"
#include "hailo_postprocessing_stage.hpp"

using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;
using InitFuncPtr = ScrfdParams *(*)(std::string, std::string);
using FreeFuncPtr = void (*)(void *);

#define NAME "hailo_scrfd"
#define POSTPROC_LIB "libscrfd_post.so"

namespace
{

std::vector<std::string> BOXES { "scrfd_2_5g/conv43", "scrfd_2_5g/conv50", "scrfd_2_5g/conv56" };
std::vector<std::string> CLASSES { "scrfd_2_5g/conv42", "scrfd_2_5g/conv49", "scrfd_2_5g/conv55" };
std::vector<std::string> LANDMARKS { "scrfd_2_5g/conv44", "scrfd_2_5g/conv51", "scrfd_2_5g/conv57" };

//******************************************************************
// BOX/LANDMARK DECODING
//******************************************************************
xt::xarray<float> decode_landmarks_scrfd(const xt::xarray<float> &landmark_detections, const xt::xarray<float> &anchors)
{
	// Decode the boxes relative to their anchors.
	// There are 5 landmarks paired in sets of 2 (x and y values),
	// so we need to tile our anchors by 5
	xt::xarray<float> landmarks =
		xt::tile(xt::view(anchors, xt::all(), xt::range(0, 2)), { 1, 5 }) +
		landmark_detections * xt::tile(xt::view(anchors, xt::all(), xt::range(2, 4)), { 1, 5 });
	return landmarks;
}

xt::xarray<float> decode_boxes_scrfd(const xt::xarray<float> &box_detections, const xt::xarray<float> &anchors)
{
	// Initalize the boxes matrix at the expected size
	xt::xarray<float> boxes = xt::zeros<float>(box_detections.shape());
	// Decode the boxes relative to their anchors in place
	xt::col(boxes, 0) = xt::col(anchors, 0) - (xt::col(box_detections, 0) * xt::col(anchors, 2));
	xt::col(boxes, 1) = xt::col(anchors, 1) - (xt::col(box_detections, 1) * xt::col(anchors, 3));
	xt::col(boxes, 2) = xt::col(anchors, 0) + (xt::col(box_detections, 2) * xt::col(anchors, 2));
	xt::col(boxes, 3) = xt::col(anchors, 1) + (xt::col(box_detections, 3) * xt::col(anchors, 3));
	return boxes;
}

std::tuple<xt::xarray<float>, xt::xarray<float>, xt::xarray<float>>
detect_decode_branch(std::map<std::string, HailoTensorPtr> &tensors, const xt::xarray<uint8_t> &boxes_quant,
					 const xt::xarray<uint8_t> &classes_quant, const xt::xarray<uint8_t> &landmarks_quant,
					 const xt::xarray<float> &anchors, const float score_threshold, const int i, const int steps)
{
	// Filter scores that pass threshold, quantize the score threshold
	auto scores_quant = xt::col(classes_quant, 1);
	xt::xarray<int> threshold_indices =
		xt::flatten_indices(xt::argwhere(scores_quant > tensors[CLASSES[i]]->quantize(score_threshold)));

	if (threshold_indices.shape(0) == 0)
		return xt::xtuple(xt::empty<float>({ 0 }), xt::empty<float>({ 0 }), xt::empty<float>({ 0 }));

	// Filter and dequantize boxes
	xt::xarray<uint8_t> high_boxes_quant = xt::view(boxes_quant, xt::keep(threshold_indices), xt::all());
	auto high_boxes_dequant = common::dequantize(high_boxes_quant,
												 tensors[BOXES[i]]->vstream_info().quant_info.qp_scale,
												 tensors[BOXES[i]]->vstream_info().quant_info.qp_zp);
	// Filter and dequantize scores
	xt::xarray<uint8_t> high_scores_quant = xt::view(scores_quant, xt::keep(threshold_indices));
	auto high_scores_dequant = common::dequantize(high_scores_quant,
												  tensors[CLASSES[i]]->vstream_info().quant_info.qp_scale,
												  tensors[CLASSES[i]]->vstream_info().quant_info.qp_zp);
	// Filter and dequantize landmarks
	xt::xarray<uint8_t> high_landmarks_quant = xt::view(landmarks_quant, xt::keep(threshold_indices), xt::all());
	auto high_landmarks_dequant = common::dequantize(high_landmarks_quant,
													 tensors[LANDMARKS[i]]->vstream_info().quant_info.qp_scale,
													 tensors[LANDMARKS[i]]->vstream_info().quant_info.qp_zp);
	// Filter anchors and use them to decode boxes/landmarks
	auto stepped_inds = threshold_indices + steps;
	auto high_anchors = xt::view(anchors, xt::keep(stepped_inds), xt::all());
	xt::xarray<float> decoded_boxes = decode_boxes_scrfd(high_boxes_dequant, high_anchors);
	xt::xarray<float> decoded_landmarks = decode_landmarks_scrfd(high_landmarks_dequant, high_anchors);

	// Return boxes, scores, and landmarks
	return xt::xtuple(decoded_boxes, high_scores_dequant, decoded_landmarks);
}

std::tuple<std::vector<xt::xarray<float>>, std::vector<xt::xarray<float>>, std::vector<xt::xarray<float>>>
detect_boxes_and_landmarks(std::map<std::string, HailoTensorPtr> &tensors,
						   const std::vector<xt::xarray<uint8_t>> &boxes_quant,
						   const std::vector<xt::xarray<uint8_t>> &classes_quant,
						   const std::vector<xt::xarray<uint8_t>> &landmarks_quant, const xt::xarray<float> &anchors,
						   const float score_threshold)
{
	std::vector<xt::xarray<float>> high_scores_dequant(CLASSES.size());
	std::vector<xt::xarray<float>> decoded_boxes(BOXES.size());
	std::vector<xt::xarray<float>> decoded_landmarks(LANDMARKS.size());

	int steps = 0;
	for (uint i = 0; i < CLASSES.size(); ++i)
	{
		auto boxes_scores_landmarks = detect_decode_branch(tensors, boxes_quant[i], classes_quant[i],
														   landmarks_quant[i], anchors, score_threshold, i, steps);
		decoded_boxes[i] = std::get<0>(boxes_scores_landmarks);
		high_scores_dequant[i] = std::get<1>(boxes_scores_landmarks);
		decoded_landmarks[i] = std::get<2>(boxes_scores_landmarks);
		steps += classes_quant[i].shape(0);
	}

	return std::tuple<std::vector<xt::xarray<float>>, std::vector<xt::xarray<float>>, std::vector<xt::xarray<float>>>(
		std::move(decoded_boxes), std::move(high_scores_dequant), std::move(decoded_landmarks));
}

//******************************************************************
// DETECTION/LANDMARKS EXTRACTION & ENCODING
//******************************************************************
void encode_detections(std::vector<HailoDetection> &objects, std::vector<xt::xarray<float>> &detection_boxes,
					   std::vector<xt::xarray<float>> &scores, std::vector<xt::xarray<float>> &landmarks)
{
	// Here we will package the processed detections into the HailoDetection meta
	// The detection meta will hold the following items:
	float confidence, w, h, xmin, ymin = 0.0f;
	// There is only 1 class in this network (face) so there is no need for label.
	std::string label = "face";
	// Iterate over our results
	for (uint i = 0; i < CLASSES.size(); ++i)
	{
		for (uint index = 0; index < scores[i].size(); ++index)
		{
			confidence = scores[i](index); // Get the score for this detection
			xmin = detection_boxes[i](index, 0); // Box xmin, relative to image size
			ymin = detection_boxes[i](index, 1); // Box ymin, relative to image size
			w = (detection_boxes[i](index, 2) - detection_boxes[i](index, 0)); // Box width, relative to image size
			h = (detection_boxes[i](index, 3) - detection_boxes[i](index, 1)); // Box height, relative to image size

			// Once all parameters are calculated, push them into the meta
			// Class = 1 since centerpose only detects people
			HailoBBox bbox(xmin, ymin, w, h);
			HailoDetection detected_face(bbox, label, confidence);

			xt::xarray<float> keypoints_raw = xt::row(landmarks[i], index);
			// The keypoints are flatten, reshape them to 2 * num_keypoints.
			int num_keypoints = keypoints_raw.shape(0) / 2;
			auto face_keypoints = xt::reshape_view(keypoints_raw, { num_keypoints, 2 });
			hailo_common::add_landmarks_to_detection(detected_face, "scrfd", face_keypoints);

			objects.push_back(detected_face); // Push the detection to the objects vector
		}
	}
}

std::vector<HailoDetection> face_detection_postprocess(std::map<std::string, HailoTensorPtr> &tensors_by_name,
													   const xt::xarray<float> &anchors, const float score_threshold,
													   const float iou_threshold, const int num_branches,
													   const int total_classes)
{
	std::vector<HailoDetection> objects; // The detection meta we will eventually return

	//-------------------------------
	// TENSOR GATHERING
	//-------------------------------
	// The output layers fall into hree categories: boxes, classes(scores), and lanmarks(x,y for each)
	std::vector<xt::xarray<uint8_t>> box_layers_quant;
	std::vector<xt::xarray<uint8_t>> class_layers_quant;
	std::vector<xt::xarray<uint8_t>> landmarks_layers_quant;

	for (uint i = 0; i < BOXES.size(); ++i)
	{
		// Extract the boxes
		xt::xarray<uint8_t> xdata_boxes = common::get_xtensor(tensors_by_name[BOXES[i]]);
		auto num_boxes = (int)xdata_boxes.shape(0) * (int)xdata_boxes.shape(1) * ((int)xdata_boxes.shape(2) / 4);
		auto xdata_boxes_reshaped =
			xt::reshape_view(xdata_boxes, { num_boxes, 4 }); // Resize to be by the 4 parameters for a box
		box_layers_quant.emplace_back(std::move(xdata_boxes_reshaped));

		// Extract the classes
		xt::xarray<uint8_t> xdata_classes = common::get_xtensor(tensors_by_name[CLASSES[i]]);
		auto num_classes =
			(int)xdata_classes.shape(0) * (int)xdata_classes.shape(1) * ((int)xdata_classes.shape(2) / total_classes);
		auto xdata_classes_reshaped = xt::reshape_view(
			xdata_classes, { num_classes, total_classes }); // Resize to be by the total_classes available classes
		class_layers_quant.emplace_back(std::move(xdata_classes_reshaped));

		// Extract the landmarks
		xt::xarray<uint8_t> xdata_landmarks = common::get_xtensor(tensors_by_name[LANDMARKS[i]]);
		auto num_landmarks =
			(int)xdata_landmarks.shape(0) * (int)xdata_landmarks.shape(1) * ((int)xdata_landmarks.shape(2) / 10);
		auto xdata_landmarks_reshaped = xt::reshape_view(
			xdata_landmarks, { num_landmarks, 10 }); // Resize to be by the (x,y) for each of the 5 landmarks (2*5=10)
		landmarks_layers_quant.emplace_back(std::move(xdata_landmarks_reshaped));
	}

	//-------------------------------
	// CALCULATION AND EXTRACTION
	//-------------------------------

	// Extract boxes and landmarks
	auto boxes_and_landmarks = detect_boxes_and_landmarks(tensors_by_name, box_layers_quant, class_layers_quant,
														  landmarks_layers_quant, anchors, score_threshold);

	// //-------------------------------
	// // RESULTS ENCODING
	// //-------------------------------

	// // Encode the individual boxes/keypoints and package them into the meta
	encode_detections(objects, std::get<0>(boxes_and_landmarks), std::get<1>(boxes_and_landmarks),
					  std::get<2>(boxes_and_landmarks));

	// // Perform nms to throw out similar detections
	common::nms(objects, iou_threshold);

	return objects;
}

//******************************************************************
//  SCRFD POSTPROCESS
//******************************************************************
void scrfd(HailoROIPtr roi, void *params_void_ptr)
{
	/*
	 *  The lightface network outputs tensors in 4 sets of 2 (totaling 8 layers).
	 *  Each set has a layer describing bounding boxes of detections, and a layer
	 *  of class scores for each corresponding box. Each set of boxes and scores
	 *  operate at a different scale of the feature set. The 4 different scales
	 *  give us decent coverage of the possible sizes objects can take in the image.
	 *  Since the detections output by the network are anchor boxes, we need to
	 *  multiply them by anchors that we determine in advanced using the parameters below.
	 */
	/*
	 *  The retinaface operates under the same principles as the lightface network
	 *  below, however here we include a set of landmarks for each detection box.
	 *  This means that instead of sets of 2 tensors, the newtork outputs 3 sets
	 *  of 3 tensors (totalling in 9 output layers). So each set has a tensor for
	 *  boxes, corresponding scores, and corresponding landmarks. Like in lightface,
	 *  we need to trabsform the boxes using anchors determined by the parameters below.
	 */
	/*
	 *  SCRFD is also a face detection + landmarks network like retinaface below.
	 *  It uses the same tensor scheme but different decoding stratedgy.
	 *  Overall differences lie in performance and resolution.
	 */
	// Get the output layers from the hailo frame.
	ScrfdParams *params = reinterpret_cast<ScrfdParams *>(params_void_ptr);
	if (!roi->has_tensors())
		return;
	std::map<std::string, HailoTensorPtr> tensors_by_name = roi->get_tensors_by_name();

	// Extract the detection objects using the given parameters.
	std::vector<HailoDetection> detections = face_detection_postprocess(
		tensors_by_name, params->anchors, params->score_threshold, params->iou_threshold, params->num_branches, 1);

	// Update the frame with the found detections.
	hailo_common::add_detections(roi, detections);
}

} // namespace

class Scrfd : public HailoPostProcessingStage
{
public:
	Scrfd(RPiCamApp *app);
	~Scrfd();

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	void runInference(const uint8_t *input, uint32_t *output);

	PostProcessingLib postproc_;
	ScrfdParams *params_;
};

Scrfd::Scrfd(RPiCamApp *app)
	: HailoPostProcessingStage(app), postproc_(PostProcLibDir(POSTPROC_LIB))
{
}

Scrfd::~Scrfd()
{
	if (params_)
	{
		FreeFuncPtr free_func = reinterpret_cast<FreeFuncPtr>(postproc_.GetSymbol("free_resources"));
		if (free_func)
			free_func(params_);
	}
}

char const *Scrfd::Name() const
{
	return NAME;
}

void Scrfd::Read(boost::property_tree::ptree const &params)
{
	InitFuncPtr init = reinterpret_cast<InitFuncPtr>(postproc_.GetSymbol("init"));
	params_ = init("", "scrfd");

	HailoPostProcessingStage::Read(params);
}

void Scrfd::Configure()
{
	HailoPostProcessingStage::Configure();
}

bool Scrfd::Process(CompletedRequestPtr &completed_request)
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

	BufferWriteSync w(app_, completed_request->buffers[output_stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *output = (uint32_t *)buffer.data();

	runInference(input_ptr, output);

	{
		Msg m(MsgType::Display, std::move(input), InputTensorSize(), "scrfd");
		msg_queue_.Clear("scrfd");
		msg_queue_.Post(std::move(m));
	}

	return false;
}

void Scrfd::runInference(const uint8_t *input, uint32_t *output)
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

	HailoROIPtr roi = MakeROI(output_tensors);
	scrfd(roi, params_);

	std::vector<HailoDetectionPtr> detections = hailo_common::get_hailo_detections(roi);
	cv::Mat image(InputTensorSize().height, InputTensorSize().width, CV_8UC3, (void *)input,
				  InputTensorSize().width * 3);

	for (auto &detection : detections)
	{
		if (detection->get_confidence() == 0)
			continue;

		HailoBBox bbox = detection->get_bbox();
		const float x0 = std::max(bbox.xmin(), 0.0f) * InputTensorSize().width;
		const float x1 = std::min(bbox.xmax(), 1.0f) * InputTensorSize().width;
		const float y0 = std::max(bbox.ymin(), 0.0f) * InputTensorSize().height;
		const float y1 = std::min(bbox.ymax(), 1.0f) * InputTensorSize().height;
		cv::rectangle(image, cv::Point2f(x0, y0), cv::Point2f(x1, y1), cv::Scalar(0, 0, 255), 1);

		std::vector<HailoLandmarksPtr> landmarks = hailo_common::get_hailo_landmarks(detection);
		if (landmarks.size() > 0)
		{
			for (auto &p : landmarks[0]->get_points())
				cv::circle(image, cv::Point(x0 + p.x() * (x1 - x0), y0 + p.y() * (y1 - y0)), 3,
						   cv::Scalar(0, 255, 0), -1);
		}
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new Scrfd(app);
}

static RegisterStage reg(NAME, &Create);
