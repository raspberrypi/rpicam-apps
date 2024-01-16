/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_mobilenet_ssd.cpp - IMX500 inference for MobileNetSSD
 */

#include <algorithm>
#include <array>
#include <cmath>
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

#define NAME "imx500_mobilenet_ssd"

// Derived from SSDMobilnetV1 DNN Model
static constexpr unsigned int TOTAL_DETECTIONS = 10;
// bbox(10 * 4) + class(10) + scores(10) + num_detections(1) = 61
static constexpr unsigned int DNN_OUTPUT_TENSOR_SIZE = 61;
static constexpr Size INPUT_TENSOR_SIZE { 300, 300 };

struct Bbox
{
	float x0;
	float y0;
	float x1;
	float y1;
};

struct ObjectDetectionSsdOutputTensor
{
	unsigned int num_detections = 0;
	std::vector<Bbox> bboxes;
	std::vector<float> scores;
	std::vector<float> classes;
};

class MobileNetSsd : public IMX500PostProcessingStage
{
public:
	MobileNetSsd(RPiCamApp *app) : IMX500PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	int processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &output_tensor,
							const Rectangle &scaler_crop) const;
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
};

char const *MobileNetSsd::Name() const
{
	return NAME;
}

void MobileNetSsd::Read(boost::property_tree::ptree const &params)
{
	max_detections_ = params.get<unsigned int>("max_detections");
	threshold_ = params.get<float>("threshold", 0.5f);

	std::string class_file = params.get<std::string>("class_file");
	std::ifstream f(class_file);
	if (f.is_open())
	{
		std::string c;
		while (std::getline(f, c))
			classes_.push_back(c);
	}
	else
		LOG_ERROR("Failed to open class file!");

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

void MobileNetSsd::Configure()
{
	// Nothing to do here except call the base class Configure().
	IMX500PostProcessingStage::Configure();
}

bool MobileNetSsd::Process(CompletedRequestPtr &completed_request)
{
	auto scaler_crop = completed_request->metadata.get(controls::ScalerCrop);
	if (!raw_stream_ || !scaler_crop)
	{
		LOG_ERROR("Must have RAW stream and scaler crop available to get sensor dimensions!");
		return false;
	}

	auto output = completed_request->metadata.get(controls::rpi::Imx500OutputTensor);
	if (!output)
	{
		LOG_ERROR("No output tensor found in metadata!");
		return false;
	}

	std::vector<float> output_tensor(output->data(), output->data() + output->size());
	std::vector<Detection> objects;

	int ret = processOutputTensor(objects, output_tensor, *scaler_crop);
	if (!ret)
	{
		if (temporal_filtering_)
		{
			// Process() can be concurrently called through different threads for consecutive CompletedRequests if
			// things are running behind.  So protect access to the lt_objects_ state object.
			std::scoped_lock<std::mutex> l(lt_lock_);

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

		if (objects.size())
			completed_request->post_process_metadata.Set("object_detect.results", objects);
	}

	IMX500PostProcessingStage::SaveInputTensor(completed_request);

	return false;
}

static int createObjectDetectionSsdData(ObjectDetectionSsdOutputTensor &ssd, const std::vector<float> &data)
{
	unsigned int count = 0;

	if ((count + (TOTAL_DETECTIONS * 4)) > DNN_OUTPUT_TENSOR_SIZE)
	{
		LOG_ERROR("Invalid count index " << count);
		return -1;
	}

	// Extract bounding box co-ordinates
	for (unsigned int i = 0; i < TOTAL_DETECTIONS; i++)
	{
		Bbox bbox;
		bbox.y0 = data.at(count + i);
		bbox.x0 = data.at(count + i + (1 * TOTAL_DETECTIONS));
		bbox.y1 = data.at(count + i + (2 * TOTAL_DETECTIONS));
		bbox.x1 = data.at(count + i + (3 * TOTAL_DETECTIONS));
		ssd.bboxes.push_back(bbox);
	}
	count += (TOTAL_DETECTIONS * 4);

	// Extract class indices
	for (unsigned int i = 0; i < TOTAL_DETECTIONS; i++)
	{
		if (count > DNN_OUTPUT_TENSOR_SIZE)
		{
			LOG_ERROR("Invalid count index " << count);
			return -1;
		}

		ssd.classes.push_back(data.at(count));
		count++;
	}

	// Extract scores
	for (unsigned int i = 0; i < TOTAL_DETECTIONS; i++)
	{
		if (count > DNN_OUTPUT_TENSOR_SIZE)
		{
			LOG_ERROR("Invalid count index " << count);
			return -1;
		}

		ssd.scores.push_back(data.at(count));
		count++;
	}

	if (count > DNN_OUTPUT_TENSOR_SIZE)
	{
		LOG_ERROR("Invalid count index " << count);
		return -1;
	}

	// Extract number of detections
	unsigned int num_detections = data.at(count);
	if (num_detections > TOTAL_DETECTIONS)
	{
		LOG(1, "Unexpected value for num_detections: " << num_detections << ", setting it to " << TOTAL_DETECTIONS);
		num_detections = TOTAL_DETECTIONS;
	}

	ssd.num_detections = num_detections;
	return 0;
}

int MobileNetSsd::processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &output_tensor,
									  const Rectangle &scaler_crop) const
{
	ObjectDetectionSsdOutputTensor ssd;

	if (output_tensor.size() != DNN_OUTPUT_TENSOR_SIZE)
	{
		LOG_ERROR("Invalid tensor size " << output_tensor.size() << ", expected " << DNN_OUTPUT_TENSOR_SIZE);
		return -1;
	}

	int ret = createObjectDetectionSsdData(ssd, output_tensor);
	if (ret)
	{
		LOG_ERROR("Failed to create SSD data");
		return -1;
	}

	for (unsigned int i = 0; i < std::min(ssd.num_detections, max_detections_); i++)
	{
		uint8_t class_index = (uint8_t)ssd.classes[i];

		// Filter detections
		if (ssd.scores[i] < threshold_ || class_index >= classes_.size())
			continue;

		// Extract bounding box co-ordinates in the inference image co-ordinates.
		int x0 = std::round((ssd.bboxes[i].x0) * (INPUT_TENSOR_SIZE.width - 1));
		int x1 = std::round((ssd.bboxes[i].x1) * (INPUT_TENSOR_SIZE.width - 1));
		int y0 = std::round((ssd.bboxes[i].y0) * (INPUT_TENSOR_SIZE.height - 1));
		int y1 = std::round((ssd.bboxes[i].y1) * (INPUT_TENSOR_SIZE.height - 1));

		// Convert the inference image co-ordinates into the final ISP output co-ordinates.
		const Rectangle obj { x0, y0, (unsigned int)x1 - x0, (unsigned int)y1 - y0 };
		const Rectangle obj_scaled = ConvertInferenceCoordinates(obj, scaler_crop, INPUT_TENSOR_SIZE);

		objects.emplace_back(class_index, classes_[class_index], ssd.scores[i],
							 obj_scaled.x, obj_scaled.y, obj_scaled.width, obj_scaled.height);
	}

	LOG(1, "Number of objects detected: " << objects.size());
	for (unsigned i = 0; i < objects.size(); i++)
		LOG(1, "[" << i << "] : " << objects[i].toString());

	return 0;
}

void MobileNetSsd::filterOutputObjects(std::vector<Detection> &objects)
{
	const Size isp_output_size = output_stream_->configuration().size;

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
		// consecutive frames.
		if (!matched)
			lt_objects_.push_back({ object, visible_frames_, hidden_frames_, 1 });
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
	return new MobileNetSsd(app);
}

static RegisterStage reg(NAME, &Create);
