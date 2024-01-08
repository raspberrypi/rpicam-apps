/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * imx500_mobilenet.cpp - IMX500 inference for MobileNet SSD
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"
#include "post_processing_stages/object_detect.hpp"
#include "post_processing_stages/post_processing_stage.hpp"


using Stream = libcamera::Stream;
namespace controls = libcamera::controls;

#define NAME "imx500_mobilenet"

// Derived from SSDMobilnetV1 DNN Model
static constexpr unsigned int TotalDetections = 10;
// bbox(10 * 4) + class(10) + scores(10) + numDetections(1) = 61
static constexpr unsigned int DnnOutputTensorSize = 61;

struct IMX500OutputTensorInfo
{
	uint32_t totalSize;
	uint32_t tensorNum;
	std::vector<float> address;
	std::vector<uint32_t> tensorDataNum;
};

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

class MobileNet : public PostProcessingStage
{
public:
	MobileNet(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	int processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &outputTensor) const;

	Stream *stream_;

	// Config params
	unsigned int maxDetections_;
	float threshold_;
	std::vector<std::string> classes_;
};

char const *MobileNet::Name() const
{
	return NAME;
}

void MobileNet::Read(boost::property_tree::ptree const &params)
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
}

void MobileNet::Configure()
{
	stream_ = app_->GetMainStream();
}

bool MobileNet::Process(CompletedRequestPtr &completed_request)
{
	auto output = completed_request->metadata.get(controls::rpi::Imx500OutputTensor);
	if (!output)
	{
		LOG_ERROR("No output tensor found in metadata!");
		return false;
	}

	std::vector<float> outputTensor(output->data(), output->data() + output->size());
	std::vector<Detection> objects;

	int ret = processOutputTensor(objects, outputTensor);
	if (!ret && objects.size())
		completed_request->post_process_metadata.Set("object_detect.results", objects);

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

int MobileNet::processOutputTensor(std::vector<Detection> &objects, const std::vector<float> &outputTensor) const
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

	const libcamera::Size dim = stream_->configuration().size;
	for (unsigned int i = 0; i < std::min(ssd.numDetections, maxDetections_); i++)
	{
		uint8_t classIndex = (uint8_t)ssd.classes[i];

		// Filter detections
		if (ssd.scores[i] < threshold_ || classIndex >= classes_.size())
			continue;

		// Extract bounding box co-ordinates
		unsigned int x0 = std::round((ssd.bboxes[i].x0) * (dim.width - 1));
		unsigned int x1 = std::round((ssd.bboxes[i].x1) * (dim.width - 1));
		unsigned int y0 = std::round((ssd.bboxes[i].y0) * (dim.height - 1));
		unsigned int y1 = std::round((ssd.bboxes[i].y1) * (dim.height - 1));

		objects.emplace_back(classIndex, classes_[classIndex], ssd.scores[i], x0, y0, x1 - x0, y1 - y0);
	}

	LOG(1, "Number of objects detected: " << objects.size());
	for (unsigned i = 0; i < objects.size(); i++)
		LOG(2, "[" << i << "] : " << objects[i].toString());

	return 0;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new MobileNet(app);
}

static RegisterStage reg(NAME, &Create);
