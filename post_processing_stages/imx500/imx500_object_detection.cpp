/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_object_detection.cpp - IMX500 object detection for various networks
 */

#include <algorithm>
#include <array>
#include <mutex>
#include <string>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/geometry.h>

#include "core/rpicam_app.hpp"
#include "post_processing_stages/object_detect.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

#include "imx500_post_processing_stage.hpp"

using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;
namespace controls = libcamera::controls;

#define NAME "imx500_object_detection"

struct Bbox
{
	float x0;
	float y0;
	float x1;
	float y1;
};

struct ObjectDetectionOutput
{
	unsigned int num_detections = 0;
	std::vector<Bbox> bboxes;
	std::vector<float> scores;
	std::vector<float> classes;
};

class ObjectDetection : public IMX500PostProcessingStage
{
public:
	ObjectDetection(RPiCamApp *app) : IMX500PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	int processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &output_tensor,
							const CnnOutputTensorInfo &output_tensor_info, const Rectangle &scaler_crop) const;
	void filterOutputObjects(std::vector<Detection> &objects);

	struct LtObject
	{
		Detection params;
		unsigned int visible;
		unsigned int hidden;
		bool matched;
	};

	std::vector<LtObject> lt_objects_;
	std::mutex lt_lock_;

	// Config params
	unsigned int max_detections_;
	float threshold_;
	std::vector<std::string> classes_;
	bool temporal_filtering_;
	float tolerance_;
	float factor_;
	unsigned int visible_frames_;
	unsigned int hidden_frames_;
	bool started_ = false;
};

char const *ObjectDetection::Name() const
{
	return NAME;
}

void ObjectDetection::Read(boost::property_tree::ptree const &params)
{
	max_detections_ = params.get<unsigned int>("max_detections");
	threshold_ = params.get<float>("threshold", 0.5f);
	classes_ = PostProcessingStage::GetJsonArray<std::string>(params, "classes");

	if (params.find("temporal_filter") != params.not_found())
	{
		temporal_filtering_ = true;
		tolerance_ = params.get<float>("temporal_filter.tolerance", 0.05);
		factor_ = params.get<float>("temporal_filter.factor", 0.2);
		visible_frames_ = params.get<unsigned int>("temporal_filter.visible_frames", 5);
		hidden_frames_ = params.get<unsigned int>("temporal_filter.hidden_frames", 2);
	}
	else
		temporal_filtering_ = false;

	IMX500PostProcessingStage::Read(params);
}

void ObjectDetection::Configure()
{
	lt_objects_.clear();
	IMX500PostProcessingStage::Configure();
	if (!started_)
	{
		IMX500PostProcessingStage::ShowFwProgressBar();
		started_ = true;
	}
}

bool ObjectDetection::Process(CompletedRequestPtr &completed_request)
{
	auto scaler_crop = completed_request->metadata.get(controls::ScalerCrop);
	if (!raw_stream_ || !scaler_crop)
	{
		LOG_ERROR("Must have RAW stream and scaler crop available to get sensor dimensions!");
		return false;
	}

	auto output = completed_request->metadata.get(controls::rpi::CnnOutputTensor);
	auto info = completed_request->metadata.get(controls::rpi::CnnOutputTensorInfo);
	std::vector<Detection> objects;

	// Process() can be concurrently called through different threads for consecutive CompletedRequests if
	// things are running behind.  So protect access to the lt_objects_ state object.
	std::scoped_lock<std::mutex> l(lt_lock_);

	if (output && info)
	{
		std::vector<float> output_tensor(output->data(), output->data() + output->size());
		CnnOutputTensorInfo output_tensor_info = *reinterpret_cast<const CnnOutputTensorInfo *>(info->data());

		processOutputTensor(objects, output_tensor, output_tensor_info, *scaler_crop);

		if (temporal_filtering_)
		{
			filterOutputObjects(objects);
			if (lt_objects_.size())
			{
				objects.clear();
				for (auto const &obj : lt_objects_)
				{
					if (!obj.hidden)
						objects.push_back(obj.params);
				}
			}
		}
		else
		{
			// If no temporal filtering, make sure we fill lt_objects_ with the current result as it may be used if
			// there is no output tensor on subsequent frames.
			lt_objects_.clear();
			for (auto const &obj : objects)
				lt_objects_.push_back({ obj, 0, 0, 0 });
		}
	}
	else
	{
		// No output tensor, so simply reuse the results from the lt_objects_.
		for (auto const &obj : lt_objects_)
		{
			if (!obj.hidden)
				objects.push_back(obj.params);
		}
	}

	if (objects.size())
		completed_request->post_process_metadata.Set("object_detect.results", objects);

	return IMX500PostProcessingStage::Process(completed_request);
}

static int createObjectDetectionData(ObjectDetectionOutput &output, const std::vector<float> &data,
									 unsigned int total_detections)
{
	unsigned int count = 0;

	// Extract bounding box co-ordinates
	for (unsigned int i = 0; i < total_detections; i++)
	{
		Bbox bbox;
		bbox.y0 = data.at(count + i);
		bbox.x0 = data.at(count + i + (1 * total_detections));
		bbox.y1 = data.at(count + i + (2 * total_detections));
		bbox.x1 = data.at(count + i + (3 * total_detections));
		output.bboxes.push_back(bbox);
	}
	count += (total_detections * 4);

	// Extract scores
	for (unsigned int i = 0; i < total_detections; i++)
	{
		output.scores.push_back(data.at(count));
		count++;
	}

	// Extract class indices
	for (unsigned int i = 0; i < total_detections; i++)
	{
		output.classes.push_back(data.at(count));
		count++;
	}

	// Extract number of detections
	unsigned int num_detections = data.at(count);
	if (num_detections > total_detections)
	{
		LOG(1, "Unexpected value for num_detections: " << num_detections << ", setting it to " << total_detections);
		num_detections = total_detections;
	}

	output.num_detections = num_detections;
	return 0;
}

int ObjectDetection::processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &output_tensor,
										 const CnnOutputTensorInfo &output_tensor_info,
										 const Rectangle &scaler_crop) const
{
	if (output_tensor_info.num_tensors != 4)
	{
		LOG_ERROR("Invalid number of tensors " << output_tensor_info.num_tensors << ", expected 4");
		return -1;
	}

	const unsigned int total_detections = output_tensor_info.info[0].tensor_data_num / 4;
	ObjectDetectionOutput output;

	// 4x coords + 1x labels + 1x confidences + 1 total detections
	if (output_tensor.size() != 6 * total_detections + 1)
	{
		LOG_ERROR("Invalid tensor size " << output_tensor.size() << ", expected " << 6 * total_detections + 1);
		return -1;
	}

	int ret = createObjectDetectionData(output, output_tensor, total_detections);
	if (ret)
	{
		LOG_ERROR("Failed to create object detection data");
		return -1;
	}

	for (unsigned int i = 0; i < std::min(output.num_detections, max_detections_); i++)
	{
		uint8_t class_index = (uint8_t)output.classes[i];

		// Filter detections
		if (output.scores[i] < threshold_ || class_index >= classes_.size())
			continue;

		// Extract bounding box co-ordinates in the inference image co-ordinates and convert to the final ISP output
		// co-ordinates.
		std::vector<float> coords{ output.bboxes[i].x0, output.bboxes[i].y0,
								   output.bboxes[i].x1 - output.bboxes[i].x0,
								   output.bboxes[i].y1 - output.bboxes[i].y0 };
		const Rectangle obj_scaled = ConvertInferenceCoordinates(coords, scaler_crop);

		objects.emplace_back(class_index, classes_[class_index], output.scores[i],
							 obj_scaled.x, obj_scaled.y, obj_scaled.width, obj_scaled.height);
	}

	LOG(2, "Number of objects detected: " << objects.size());
	for (unsigned i = 0; i < objects.size(); i++)
		LOG(2, "[" << i << "] : " << objects[i].toString());

	return 0;
}

void ObjectDetection::filterOutputObjects(std::vector<Detection> &objects)
{
	const Size isp_output_size = output_stream_->configuration().size;
	bool empty = lt_objects_.empty();

	for (auto &lt_obj : lt_objects_)
		lt_obj.matched = false;

	for (auto const &object : objects)
	{
		bool matched = false;
		for (auto &lt_obj : lt_objects_)
		{
			// Try and match a detected object in our long term list.
			if (object.category == lt_obj.params.category &&
				std::abs(object.box.x - lt_obj.params.box.x) < tolerance_ * isp_output_size.width &&
				std::abs(object.box.y - lt_obj.params.box.y) < tolerance_ * isp_output_size.height &&
				std::abs((int)object.box.width - (int)lt_obj.params.box.width) < tolerance_ * isp_output_size.width &&
				std::abs((int)object.box.height - (int)lt_obj.params.box.height) < tolerance_ * isp_output_size.height)
			{
				lt_obj.matched = matched = true;
				lt_obj.params.confidence = object.confidence;
				lt_obj.params.box.x = factor_ * object.box.x + (1 - factor_) * lt_obj.params.box.x;
				lt_obj.params.box.y = factor_ * object.box.y + (1 - factor_) * lt_obj.params.box.y;
				lt_obj.params.box.width = factor_ * object.box.width + (1 - factor_) * lt_obj.params.box.width;
				lt_obj.params.box.height = factor_ * object.box.height + (1 - factor_) * lt_obj.params.box.height;
				// Reset the visibility counter for when the object next disappears.
				lt_obj.visible = visible_frames_;
				// Decrement the hidden counter until the object becomes visible in the list.
				lt_obj.hidden = std::max(0, (int)lt_obj.hidden - 1);
				break;
			}
		}

		// Add the object to the long term list if not found.  This object will remain hidden for hidden_frames_
		// consecutive frames unless the long term list started out empty.
		if (!matched)
		{
			const unsigned int hidden_frames = empty ? 0 : hidden_frames_;
			lt_objects_.push_back({ object, visible_frames_, hidden_frames, 1 });
		}
	}

	for (auto &lt_obj : lt_objects_)
	{
		if (!lt_obj.matched)
		{
			// If a non matched object in the long term list is still hidden, set visible count to 0 so that it must be
			// matched for hidden_frames_ consecutive frames before becoming visible. Otherwise, decrement the visible
			// count of unmatched objects in the long term list.
			if (lt_obj.hidden)
				lt_obj.visible = 0;
			else
				lt_obj.visible--;
		}
	}

	// Remove now invisible objects from the long term list.
	lt_objects_.erase(std::remove_if(lt_objects_.begin(), lt_objects_.end(),
						[] (const LtObject &obj) { return !obj.matched && !obj.visible; }),
					  lt_objects_.end());
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectDetection(app);
}

static RegisterStage reg(NAME, &Create);
