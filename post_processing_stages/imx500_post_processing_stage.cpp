/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_post_rpocessing_stage.cpp - IMX500 post processing stage base class
 */

#include "imx500_post_processing_stage.hpp"

#include <libcamera/control_ids.h>

using Stream = libcamera::Stream;
using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;

void IMX500PostProcessingStage::Read(boost::property_tree::ptree const &params)
{
	if (params.find("save_input_tensor") != params.not_found())
	{
		std::string filename = params.get<std::string>("save_input_tensor.filename");
		numInputTensorsSaved_ = params.get<unsigned int>("save_input_tensor.num_tensors", 1);
		inputTensorFile_ = std::ofstream(filename, std::ios::out | std::ios::binary);
	}
}

void IMX500PostProcessingStage::Configure()
{
	outputStream_ = app_->GetMainStream();
	rawStream_ = app_->RawStream();
	saveFrames_ = numInputTensorsSaved_;
	fullSensorResolution_ = *app_->GetProperties().get(properties::ScalerCropMaximum);
}

void IMX500PostProcessingStage::SaveInputTensor(CompletedRequestPtr &completed_request)
{
	auto input = completed_request->metadata.get(controls::rpi::Imx500InputTensor);
	if (input && inputTensorFile_.is_open())
	{
		inputTensorFile_.write(reinterpret_cast<const char *>(input->data()), input->size());
		if (--saveFrames_ == 0)
			inputTensorFile_.close();
	}
}

Rectangle IMX500PostProcessingStage::ConvertInferenceCoordinates(const Rectangle &obj, const Rectangle &scalerCrop,
																 const Size &inputTensorSize) const
{
	// Convert the inference image co-ordinates into the final ISP output co-ordinates.
	const Size &ispOutputSize = outputStream_->configuration().size;
	const Size &sensorOutputSize = rawStream_->configuration().size;
	const Rectangle sensorCrop = scalerCrop.scaledBy(sensorOutputSize, fullSensorResolution_.size());

	// Object on inference image -> sensor image
	const Rectangle objSensor = obj.scaledBy(sensorOutputSize, inputTensorSize);
	// -> bounded to the ISP crop on the sensor image
	const Rectangle objBound = objSensor.boundedTo(sensorCrop);
	// -> translated by the start of the crop offset
	const Rectangle objTranslated = objBound.translatedBy(-sensorCrop.topLeft());
	// -> and finally scaled to the ISP output.
	const Rectangle objScaled = objTranslated.scaledBy(ispOutputSize, sensorOutputSize);

	LOG(2, obj << " -> (sensor) " << objSensor << " -> (bound) " << objBound
			   << " -> (translate) " << objTranslated << " -> (scaled) " << objScaled);
	
	return objScaled;
}
