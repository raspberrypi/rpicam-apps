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
		num_input_tensors_saved_ = params.get<unsigned int>("save_input_tensor.num_tensors", 1);
		input_tensor_signed_ = params.get<bool>("save_input_tensor.is_signed");
		input_tensor_file_ = std::ofstream(filename, std::ios::out | std::ios::binary);
	}
}

void IMX500PostProcessingStage::Configure()
{
	output_stream_ = app_->GetMainStream();
	raw_stream_ = app_->RawStream();
	save_frames_ = num_input_tensors_saved_;
	full_sensor_resolution_ = *app_->GetProperties().get(properties::ScalerCropMaximum);
}

bool IMX500PostProcessingStage::Process(CompletedRequestPtr &completed_request)
{
	auto input = completed_request->metadata.get(controls::rpi::Imx500InputTensor);

	if (input && input_tensor_file_.is_open())
	{
		// There is a chance that this may be called through multiple threads, so serialize the file access.
		std::scoped_lock<std::mutex> l(lock_);

		if (input_tensor_signed_)
		{
			for (unsigned int i = 0; i < input->size(); i++)
			{
				int16_t sample = static_cast<int8_t>(input->data()[i]);
				sample = std::clamp<int16_t>(sample + 128, 0, 255);
				input_tensor_file_.put(static_cast<uint8_t>(sample));
			}
		}
		else
			input_tensor_file_.write(reinterpret_cast<const char *>(input->data()), input->size());

		if (--save_frames_ == 0)
			input_tensor_file_.close();
	}

	return false;
}

Rectangle IMX500PostProcessingStage::ConvertInferenceCoordinates(const Rectangle &obj, const Rectangle &scaler_crop,
																 const Size &input_tensor_size) const
{
	// Convert the inference image co-ordinates into the final ISP output co-ordinates.
	const Size &isp_output_size = output_stream_->configuration().size;
	const Size &sensor_output_size = raw_stream_->configuration().size;
	const Rectangle sensor_crop = scaler_crop.scaledBy(sensor_output_size, full_sensor_resolution_.size());

	// Object on inference image -> sensor image
	const Rectangle obj_sensor = obj.scaledBy(sensor_output_size, input_tensor_size);
	// -> bounded to the ISP crop on the sensor image
	const Rectangle obj_bound = obj_sensor.boundedTo(sensor_crop);
	// -> translated by the start of the crop offset
	const Rectangle obc_translated = obj_bound.translatedBy(-sensor_crop.topLeft());
	// -> and finally scaled to the ISP output.
	const Rectangle obj_scaled = obc_translated.scaledBy(isp_output_size, sensor_output_size);

	LOG(2, obj << " -> (sensor) " << obj_sensor << " -> (bound) " << obj_bound
			   << " -> (translate) " << obc_translated << " -> (scaled) " << obj_scaled);
	
	return obj_scaled;
}
