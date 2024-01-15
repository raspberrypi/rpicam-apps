/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_post_rpocessing_stage.hpp - IMX500 post processing stage base class
 */

#pragma once

#include <fstream>

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
	libcamera::Rectangle fullSensorResolution_;
	libcamera::Stream *outputStream_;
	libcamera::Stream *rawStream_;

private:
	std::ofstream inputTensorFile_;
	unsigned int numInputTensorsSaved_;
	unsigned int saveFrames_;
};
