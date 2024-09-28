/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_post_rpocessing_stage.cpp - IMX500 post processing stage base class
 */
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <filesystem>
#include <linux/videodev2.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

#include <boost/property_tree/ptree.hpp>

#include "imx500_post_processing_stage.hpp"

#include <libcamera/control_ids.h>

using Stream = libcamera::Stream;
using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;

using namespace std::chrono_literals;

namespace fs = std::filesystem;

namespace
{

const unsigned int ROI_CTRL_ID = 0x00982900;
const unsigned int NETWORK_FW_CTRL_ID = 0x00982901;

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

				/* Find the progress indicator sysfs dev nodes. */
				const std::string test_dir_str = fs::read_symlink(test_dir).string();
				const std::size_t pos = test_dir_str.find_last_of("/") + 1;
				assert(pos != std::string::npos);

				const std::string imx500_device_id = test_dir_str.substr(pos);
				std::string spi_device_id = imx500_device_id;
				const std::size_t rep = spi_device_id.find("001a");
				spi_device_id.replace(rep, 4, "0040");

				const fs::path imx500_progress { "/sys/kernel/debug/imx500-fw:" + imx500_device_id + "/fw_progress" };
				const fs::path spi_progress { "/sys/kernel/debug/rp2040-spi:" + spi_device_id + "/transfer_progress" };

				fw_progress_.open(imx500_progress.c_str(), std::ios_base::in);
				fw_progress_chunk_.open(spi_progress.c_str(), std::ios_base::in);

				break;
			}
		}
	}

	if (device_fd_ < 0)
		throw std::runtime_error("Cannot open imx500 device node");

	SetInferenceRoiAbs({ 0, 0, 4056, 3040 });
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

void IMX500PostProcessingStage::SetInferenceRoiAbs(const Rectangle &roi_) const
{
	Rectangle roi = roi_.boundedTo(full_sensor_resolution_);
	const uint32_t roi_array[4] = { (uint32_t)roi.x, (uint32_t)roi.y, (uint32_t)roi.width, (uint32_t)roi.height };

	v4l2_ext_control roi_ctrl;
	roi_ctrl.id = ROI_CTRL_ID;
	roi_ctrl.p_u32 = (unsigned int *)&roi_array;
	roi_ctrl.size = 16;

	v4l2_ext_controls ctrls;
	ctrls.count = 1;
	ctrls.controls = (v4l2_ext_control *)&roi_ctrl;

	int ret = ioctl(device_fd_, VIDIOC_S_EXT_CTRLS, &ctrls);
	if (ret)
		LOG_ERROR("IMX500: Unable to set absolute ROI");
}

void IMX500PostProcessingStage::SetInferenceRoiAuto(const unsigned int width, const unsigned int height) const
{
	Size s = full_sensor_resolution_.size().boundedToAspectRatio(Size(width, height));
	Rectangle r = s.centeredTo(full_sensor_resolution_.center()).enclosedIn(full_sensor_resolution_);
	SetInferenceRoiAbs(r);
}

void IMX500PostProcessingStage::ShowFwProgressBar()
{
	if (fw_progress_.is_open() && fw_progress_chunk_.is_open())
	{
		std::thread progress_thread { &IMX500PostProcessingStage::doProgressBar, this };
		progress_thread.detach();
	}
}

std::vector<unsigned int> split(std::stringstream &stream)
{
	std::vector<unsigned int> result;
	unsigned int n;

	while (stream >> n)
		result.push_back(n);

	return result;
}

void IMX500PostProcessingStage::doProgressBar()
{
	while (1)
	{
		std::stringstream fw_progress_str, block_str;
		unsigned int block_progress;

		fw_progress_.seekg(0);
		fw_progress_chunk_.seekg(0);
		fw_progress_str << fw_progress_.rdbuf();
		block_str << fw_progress_chunk_.rdbuf();

		std::vector<unsigned int> progress = split(fw_progress_str);
		block_str >> block_progress;

		// [0] == FW state, [1] == current size, [2] == total size.
		if (progress.size() == 3 && progress[0] == 2)
		{
			unsigned int current = progress[1] + block_progress;
			std::cerr << "Network Firmware Upload: " << current * 100 / progress[2] << "% (" << current / 1024 << "/"
					  << progress[2] / 1024 << " KB)\r";
			std::cerr.flush();

			if (progress[2] && progress[1] == progress[2])
			{
				std::cerr << std::endl;
				break;
			}
		}

		std::this_thread::sleep_for(500ms);
	}
}
