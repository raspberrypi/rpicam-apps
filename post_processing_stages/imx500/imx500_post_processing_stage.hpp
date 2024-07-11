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

	static constexpr unsigned int max_num_tensors = 8;
	static constexpr unsigned int network_name_len = 64;

	struct OutputTensorInfo
	{
		uint32_t tensor_data_num;
		uint16_t size;
		uint8_t ordinal;
		uint8_t serialization_index;
	};

	struct CnnOutputTensorInfo
	{
		char network_name[network_name_len];
		uint32_t num_tensors;
		OutputTensorInfo info[max_num_tensors];
	};

	IMX500PostProcessingStage(RPiCamApp *app);
	~IMX500PostProcessingStage();

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	libcamera::Rectangle ConvertInferenceCoordinates(const std::vector<float> &coords,
													 const libcamera::Rectangle &scalerCrop) const;

protected:
	libcamera::Rectangle full_sensor_resolution_;
	libcamera::Stream *output_stream_;
	libcamera::Stream *raw_stream_;

private:
	int device_fd_;
	std::ofstream input_tensor_file_;
	unsigned int num_input_tensors_saved_;
	unsigned int save_frames_;
	std::vector<int32_t> norm_val_;
	std::vector<uint8_t> norm_shift_;
	std::vector<int16_t> div_val_;
	unsigned int div_shift_;
	std::mutex lock_;
};
