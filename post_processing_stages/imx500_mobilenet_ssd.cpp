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
static constexpr unsigned int TotalDetections = 10;
// bbox(10 * 4) + class(10) + scores(10) + numDetections(1) = 61
static constexpr unsigned int DnnOutputTensorSize = 61;
static constexpr Size InputTensorSize { 300, 300 };

struct Bbox
{
	float x0;
	float y0;
	float x1;
	float y1;
};

struct ObjectDetectionSsdOutputTensor
{
	unsigned int numDetections = 0;
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
	int processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &outputTensor,
							const Rectangle &scalerCrop) const;
	void filterOutputObjects(std::vector<Detection> &objects);

	struct LtObject
	{
		Detection params;
		unsigned int visible;
		bool matched;
	};

	std::vector<LtObject> ltObjects_;
	std::mutex ltLock_;

	// Config params
	unsigned int maxDetections_;
	float threshold_;
	std::vector<std::string> classes_;
	unsigned int numInputTensorsSaved_;
	bool temporalFiltering_;
	float tolerance_;
	float factor_;
	unsigned int visibleFrames_;
};

char const *MobileNetSsd::Name() const
{
	return NAME;
}

void MobileNetSsd::Read(boost::property_tree::ptree const &params)
{
	maxDetections_ = params.get<unsigned int>("max_detections");
	threshold_ = params.get<float>("threshold", 0.5f);

	std::string classFile = params.get<std::string>("class_file");
	std::ifstream f(classFile);
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
		temporalFiltering_ = true;
		tolerance_ = params.get<float>("temporal_filter.tolerance", 0.05);
		factor_ = params.get<float>("temporal_filter.factor", 0.2);
		visibleFrames_ = params.get<unsigned int>("temporal_filter.visible_frames", 5);
	}
	else
		temporalFiltering_ = false;

	IMX500PostProcessingStage::Read(params);
}

void MobileNetSsd::Configure()
{
	// Nothing to do here except call the base class Configure().
	IMX500PostProcessingStage::Configure();
}

bool MobileNetSsd::Process(CompletedRequestPtr &completed_request)
{
	auto scalerCrop = completed_request->metadata.get(controls::ScalerCrop);
	if (!rawStream_ || !scalerCrop)
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

	std::vector<float> outputTensor(output->data(), output->data() + output->size());
	std::vector<Detection> objects;

	int ret = processOutputTensor(objects, outputTensor, *scalerCrop);
	if (!ret)
	{
		if (temporalFiltering_)
		{
			// Process() can be concurrently called through different threads for consecutive CompletedRequests if
			// things are running behind.  So protect access to the ltObjects_ state object.
			std::scoped_lock<std::mutex> l(ltLock_);
			std::vector<Detection> ltObjs;

			filterOutputObjects(objects);
			if (ltObjects_.size())
			{
				objects.clear();
				for (auto const &obj : ltObjects_)
					objects.push_back(obj.params);
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

	if ((count + (TotalDetections * 4)) > DnnOutputTensorSize)
	{
		LOG_ERROR("Invalid count index " << count);
		return -1;
	}

	// Extract bounding box co-ordinates
	for (unsigned int i = 0; i < TotalDetections; i++)
	{
		Bbox bbox;
		bbox.y0 = data.at(count + i);
		bbox.x0 = data.at(count + i + (1 * TotalDetections));
		bbox.y1 = data.at(count + i + (2 * TotalDetections));
		bbox.x1 = data.at(count + i + (3 * TotalDetections));
		ssd.bboxes.push_back(bbox);
	}
	count += (TotalDetections * 4);

	// Extract class indices
	for (unsigned int i = 0; i < TotalDetections; i++)
	{
		if (count > DnnOutputTensorSize)
		{
			LOG_ERROR("Invalid count index " << count);
			return -1;
		}

		ssd.classes.push_back(data.at(count));
		count++;
	}

	// Extract scores
	for (unsigned int i = 0; i < TotalDetections; i++)
	{
		if (count > DnnOutputTensorSize)
		{
			LOG_ERROR("Invalid count index " << count);
			return -1;
		}

		ssd.scores.push_back(data.at(count));
		count++;
	}

	if (count > DnnOutputTensorSize)
	{
		LOG_ERROR("Invalid count index " << count);
		return -1;
	}

	// Extract number of detections
	unsigned int numDetections = data.at(count);
	if (numDetections > TotalDetections)
	{
		LOG(1, "Unexpected value for numDetections: " << numDetections << ", setting it to " << TotalDetections);
		numDetections = TotalDetections;
	}

	ssd.numDetections = numDetections;
	return 0;
}

int MobileNetSsd::processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &outputTensor,
									  const Rectangle &scalerCrop) const
{
	ObjectDetectionSsdOutputTensor ssd;

	if (outputTensor.size() != DnnOutputTensorSize)
	{
		LOG_ERROR("Invalid tensor size " << outputTensor.size() << ", expected " << DnnOutputTensorSize);
		return -1;
	}

	int ret = createObjectDetectionSsdData(ssd, outputTensor);
	if (ret)
	{
		LOG_ERROR("Failed to create SSD data");
		return -1;
	}

	for (unsigned int i = 0; i < std::min(ssd.numDetections, maxDetections_); i++)
	{
		uint8_t classIndex = (uint8_t)ssd.classes[i];

		// Filter detections
		if (ssd.scores[i] < threshold_ || classIndex >= classes_.size())
			continue;

		// Extract bounding box co-ordinates in the inference image co-ordinates.
		int x0 = std::round((ssd.bboxes[i].x0) * (InputTensorSize.width - 1));
		int x1 = std::round((ssd.bboxes[i].x1) * (InputTensorSize.width - 1));
		int y0 = std::round((ssd.bboxes[i].y0) * (InputTensorSize.height - 1));
		int y1 = std::round((ssd.bboxes[i].y1) * (InputTensorSize.height - 1));

		// Convert the inference image co-ordinates into the final ISP output co-ordinates.
		const Rectangle obj { x0, y0, (unsigned int)x1 - x0, (unsigned int)y1 - y0 };
		const Rectangle objScaled = ConvertInferenceCoordinates(obj, scalerCrop, InputTensorSize);

		objects.emplace_back(classIndex, classes_[classIndex], ssd.scores[i],
							 objScaled.x, objScaled.y, objScaled.width, objScaled.height);
	}

	LOG(1, "Number of objects detected: " << objects.size());
	for (unsigned i = 0; i < objects.size(); i++)
		LOG(1, "[" << i << "] : " << objects[i].toString());

	return 0;
}

void MobileNetSsd::filterOutputObjects(std::vector<Detection> &objects)
{
	const Size ispOutputSize = outputStream_->configuration().size;

	for (auto &ltObject : ltObjects_)
		ltObject.matched = false;

	for (auto const &object : objects)
	{
		bool matched = false;
		for (auto &ltObject : ltObjects_)
		{
			// Try and match a detected object in our long term list.
			if (object.category == ltObject.params.category &&
				std::abs(object.box.x - ltObject.params.box.x) < tolerance_ * ispOutputSize.width &&
				std::abs(object.box.y - ltObject.params.box.y) < tolerance_ * ispOutputSize.height &&
				std::abs((int)object.box.width - (int)ltObject.params.box.width) < tolerance_ * ispOutputSize.width &&
				std::abs((int)object.box.height - (int)ltObject.params.box.height) < tolerance_ * ispOutputSize.height)
			{
				ltObject.matched = true;
				ltObject.visible = visibleFrames_;
				ltObject.params.confidence = object.confidence;
				ltObject.params.box.x = factor_ * object.box.x + (1 - factor_) * ltObject.params.box.x;
				ltObject.params.box.y = factor_ * object.box.y + (1 - factor_) * ltObject.params.box.y;
				ltObject.params.box.width = factor_ * object.box.width + (1 - factor_) * ltObject.params.box.width;
				ltObject.params.box.height = factor_ * object.box.height + (1 - factor_) * ltObject.params.box.height;
				matched = true;
				break;
			}
		}

		// Add the object to the long term list if not found.
		if (!matched)
			ltObjects_.push_back({ object, visibleFrames_, 1 });
	}

	// Decrement the visible count of unmatched objects in the long term list.
	for (auto &ltObject : ltObjects_)
	{
		if (!ltObject.matched)
			ltObject.visible--;
	}

	// Remove now invisible objects from the long term list.
	ltObjects_.erase(std::remove_if(ltObjects_.begin(), ltObjects_.end(),
						[] (const LtObject &obj) { return !obj.matched && !obj.visible; }),
					 ltObjects_.end());
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new MobileNetSsd(app);
}

static RegisterStage reg(NAME, &Create);
