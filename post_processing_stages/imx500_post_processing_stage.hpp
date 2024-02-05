/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_post_rpocessing_stage.hpp - IMX500 post processing stage base class
 */

#pragma once

#include <fstream>
#include <mutex>

#include <boost/property_tree/ptree.hpp>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "core/completed_request.hpp"
#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

class IMX500PostProcessingStage : public PostProcessingStage
{
public:
	IMX500PostProcessingStage(RPiCamApp *app) : PostProcessingStage(app) {}

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	void SaveInputTensor(CompletedRequestPtr &completed_request);

	libcamera::Rectangle ConvertInferenceCoordinates(const libcamera::Rectangle &obj,
													 const libcamera::Rectangle &scalerCrop,
													 const libcamera::Size &inputTensorSize) const;

protected:
	libcamera::Rectangle full_sensor_resolution_;
	libcamera::Stream *output_stream_;
	libcamera::Stream *raw_stream_;

private:
	std::ofstream input_tensor_file_;
	unsigned int num_input_tensors_saved_;
	unsigned int save_frames_;
	bool input_tensor_signed_;
	std::mutex lock_;
};
