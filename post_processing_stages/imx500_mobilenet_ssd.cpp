/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * imx500_mobilenet_ssd.cpp - IMX500 inference for MobileNetSsd SSD
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"
#include "post_processing_stages/object_detect.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

using Stream = libcamera::Stream;
using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;
namespace controls = libcamera::controls;

#define NAME "imx500_mobilenet_ssd"

// Derived from SSDMobilnetV1 DNN Model
static constexpr unsigned int TotalDetections = 10;
// bbox(10 * 4) + class(10) + scores(10) + numDetections(1) = 61
static constexpr unsigned int DnnOutputTensorSize = 61;
static constexpr Size InputTensorSize { 300, 300 };
static constexpr Size FullSensorSize { 4056, 3040 };

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

class MobileNetSsd : public PostProcessingStage
{
public:
	MobileNetSsd(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	int processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &outputTensor,
							const Rectangle &scalerCrop) const;

	Stream *outputStream_;
	Stream *rawStream_;
	std::ofstream inputTensorFile_;
	unsigned int saveFrames_;

	// Config params
	unsigned int maxDetections_;
	float threshold_;
	std::vector<std::string> classes_;
	std::string inputTensorSaveFile_;
	unsigned int numInputTensorsSaved_;
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

	std::string inputTensorSaveFile_ = params.get<std::string>("input_tensor_save_file", "");
	if (!inputTensorSaveFile_.empty())
	{
		inputTensorFile_ = std::ofstream(inputTensorSaveFile_, std::ios::out | std::ios::binary);
		numInputTensorsSaved_ = params.get<unsigned int>("num_input_tensors_saved", 1);
	}
}

void MobileNetSsd::Configure()
{
	outputStream_ = app_->GetMainStream();
	rawStream_ = app_->RawStream();
	saveFrames_ = numInputTensorsSaved_;
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
	if (!ret && objects.size())
		completed_request->post_process_metadata.Set("object_detect.results", objects);

	auto input = completed_request->metadata.get(controls::rpi::Imx500InputTensor);
	if (input && inputTensorFile_.is_open() && saveFrames_)
	{
		inputTensorFile_.write(reinterpret_cast<const char *>(input->data()), input->size());
		if (--saveFrames_ == 0)
			inputTensorFile_.close();
	}

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
		const Size ispOutputSize = outputStream_->configuration().size;
		const Size sensorOutputSize = rawStream_->configuration().size;
		Rectangle sensorCrop = scalerCrop.scaledBy(sensorOutputSize, FullSensorSize);

		// Object on inference image
		const Rectangle obj { x0, y0, (unsigned int)x1 - x0, (unsigned int)y1 - y0 };
		// -> on sensor image
		const Rectangle objSensor = obj.scaledBy(sensorOutputSize, InputTensorSize);
		// -> bounded to the ISP crop on the sensor image
		const Rectangle objBound = objSensor.boundedTo(sensorCrop);
		// -> translated by the start of the crop offset
		const Rectangle objTranslated = objBound.translatedBy(-sensorCrop.topLeft());
		// -> and finally scaled to the ISP output.
		const Rectangle objScaled = objTranslated.scaledBy(ispOutputSize, sensorOutputSize);

		LOG(2, obj << " -> (sensor) " << objSensor << " -> (bound) " << objBound
				   << " -> (translate) " << objTranslated << " -> (scaled) " << objScaled);

		objects.emplace_back(classIndex, classes_[classIndex], ssd.scores[i],
							 objScaled.x, objScaled.y, objScaled.width, objScaled.height);
	}

	LOG(1, "Number of objects detected: " << objects.size());
	for (unsigned i = 0; i < objects.size(); i++)
		LOG(1, "[" << i << "] : " << objects[i].toString());

	return 0;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new MobileNetSsd(app);
}

static RegisterStage reg(NAME, &Create);
