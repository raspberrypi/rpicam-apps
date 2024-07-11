/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_post_rpocessing_stage.cpp - IMX500 post processing stage base class
 */
#include <cmath>
#include <fcntl.h>
#include <filesystem>
#include <linux/videodev2.h>
#include <string>
#include <sys/ioctl.h>

#include <boost/property_tree/ptree.hpp>

#include "imx500_post_processing_stage.hpp"

#include <libcamera/control_ids.h>

using Stream = libcamera::Stream;
using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;

namespace fs = std::filesystem;

namespace
{

const unsigned int NETWORK_FW_CTRL_ID = 0x00981b01;

inline int16_t conv_reg_signed(int16_t reg)
{
	constexpr unsigned int ROT_DNN_NORM_SIGNED_SHT = 8;
	constexpr unsigned int ROT_DNN_NORM_MASK = 0x01FF;

	if (!((reg >> ROT_DNN_NORM_SIGNED_SHT) & 1))
		return reg;
	else
		return -((~reg + 1) & ROT_DNN_NORM_MASK);
}

} // namespace


IMX500PostProcessingStage::IMX500PostProcessingStage(RPiCamApp *app)
	: PostProcessingStage(app), device_fd_(-1)
{
	for (unsigned int i = 0; i < 16; i++)
	{
		const fs::path test_dir { "/sys/class/video4linux/v4l-subdev" + std::to_string(i) + "/device" };
		const fs::path module_dir { test_dir.string() + "/driver/module" };
		const fs::path id_dir { test_dir.string() + "/of_node" };

		if (fs::exists(module_dir) && fs::is_symlink(module_dir))
		{
			fs::path ln = fs::read_symlink(module_dir);
			if (ln.string().find("imx500") != std::string::npos)
			{
				const std::string dev_node { "/dev/v4l-subdev" + std::to_string(i) };
				device_fd_ = open(dev_node.c_str(), O_RDONLY, 0);
				break;
			}
		}
	}

	if (device_fd_ < 0)
		throw std::runtime_error("Cannot open imx500 device node");
}

IMX500PostProcessingStage::~IMX500PostProcessingStage()
{
	if (device_fd_ >= 0)
		close(device_fd_);
}

void IMX500PostProcessingStage::Read(boost::property_tree::ptree const &params)
{
	if (params.find("save_input_tensor") != params.not_found())
	{
		auto const &pt = params.get_child("save_input_tensor");

		std::string filename = pt.get<std::string>("filename");
		num_input_tensors_saved_ = pt.get<unsigned int>("num_tensors", 1);
		input_tensor_file_ = std::ofstream(filename, std::ios::out | std::ios::binary);

		norm_val_ = PostProcessingStage::GetJsonArray<int32_t>(pt, "norm_val", { 0, 0, 0, 0 });
		norm_shift_ = PostProcessingStage::GetJsonArray<uint8_t>(pt, "norm_shift", { 0, 0, 0, 0 });
		div_val_ = PostProcessingStage::GetJsonArray<int16_t>(pt, "div_val", { 1, 1, 1, 1 });
		div_shift_ = pt.get<unsigned int>("div_shift", 0);
	}

	/* Load the network firmware. */
	std::string network_file = params.get<std::string>("network_file");
	if (!fs::exists(network_file))
		throw std::runtime_error(network_file + " not found!");

	int fd = open(network_file.c_str(), O_RDONLY, 0);

	v4l2_control ctrl { NETWORK_FW_CTRL_ID, fd };
	int ret = ioctl(device_fd_, VIDIOC_S_CTRL, &ctrl);
	if (ret)
		throw std::runtime_error("failed to set network fw ioctl");

	close(fd);

	LOG(1, "\n------------------------------------------------------------------------------------------------------------------\n"
		"NOTE: Loading network firmware onto the IMX500 can take several minutes, please do not close down the application."
		"\n------------------------------------------------------------------------------------------------------------------\n");
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
	auto input = completed_request->metadata.get(controls::rpi::CnnInputTensor);

	if (input && input_tensor_file_.is_open())
	{
		// There is a chance that this may be called through multiple threads, so serialize the file access.
		std::scoped_lock<std::mutex> l(lock_);

		for (unsigned int i = 0; i < input->size(); i++)
		{
			const unsigned int channel = i % 3; // Assume RGB interleaved format.
			int16_t sample = static_cast<int8_t>(input->data()[i]);
			sample = (sample << norm_shift_[channel]) - conv_reg_signed(norm_val_[channel]);
			sample = ((sample << div_shift_) / div_val_[channel]) & 0xFF;
			input_tensor_file_.put(static_cast<uint8_t>(sample));
		}

		if (--save_frames_ == 0)
			input_tensor_file_.close();
	}

	return false;
}

Rectangle IMX500PostProcessingStage::ConvertInferenceCoordinates(const std::vector<float> &coords,
																 const Rectangle &scaler_crop) const
{
	// Convert the inference image co-ordinates into the final ISP output co-ordinates.
	const Size &isp_output_size = output_stream_->configuration().size;
	const Size &sensor_output_size = raw_stream_->configuration().size;
	const Rectangle sensor_crop = scaler_crop.scaledBy(sensor_output_size, full_sensor_resolution_.size());

	if (coords.size() != 4)
		return {};

	// Object scaled to the full sensor resolution
	Rectangle obj;
	obj.x = std::round(coords[0] * (full_sensor_resolution_.width - 1));
	obj.y = std::round(coords[1] * (full_sensor_resolution_.height - 1));
	obj.width = std::round(coords[2] * (full_sensor_resolution_.width - 1));
	obj.height = std::round(coords[3] * (full_sensor_resolution_.height - 1));

	// Object on inference image -> sensor image
	const Rectangle obj_sensor = obj.scaledBy(sensor_output_size, full_sensor_resolution_.size());
	// -> bounded to the ISP crop on the sensor image
	const Rectangle obj_bound = obj_sensor.boundedTo(sensor_crop);
	// -> translated by the start of the crop offset
	const Rectangle obj_translated = obj_bound.translatedBy(-sensor_crop.topLeft());
	// -> and finally scaled to the ISP output.
	const Rectangle obj_scaled = obj_translated.scaledBy(isp_output_size, sensor_crop.size());

	LOG(2, obj << " -> (sensor) " << obj_sensor << " -> (bound) " << obj_bound
			   << " -> (translate) " << obj_translated << " -> (scaled) " << obj_scaled);
	
	return obj_scaled;
}
