/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * encoder.cpp - Video encoder class.
 */

#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/videodev2.h>

#include "core/dl_lib.hpp"

#include "encoder.hpp"
#include "h264_encoder.hpp"
#include "mjpeg_encoder.hpp"
#include "null_encoder.hpp"

#include "config.h"

namespace fs = std::filesystem;

EncoderFactory &EncoderFactory::GetInstance()
{
	static EncoderFactory instance;
	return instance;
}

void EncoderFactory::RegisterEncoder(const std::string &name, EncoderCreateFunc create_func)
{
	encoders_[name] = create_func;
}

EncoderCreateFunc EncoderFactory::CreateEncoder(const std::string &name)
{
	auto it = encoders_.find(name);
	if (it != encoders_.end())
		return it->second;
	return nullptr;
}

bool EncoderFactory::HasEncoder(const std::string &name) const
{
	return encoders_.find(name) != encoders_.end();
}

void EncoderFactory::LoadEncoderLibraries(const std::string &lib_dir)
{
	const fs::path path(!lib_dir.empty() ? lib_dir : ENCODER_LIB_DIR);
	const std::string ext(".so");

	if (!fs::exists(path))
		return;

	// Dynamically load all .so files from the system encoder lib path.
	// This will automatically register the stages with the factory.
	for (auto const &p : fs::recursive_directory_iterator(path))
	{
		if (p.path().extension() == ext)
		{
			const std::string library_path = p.path().string();

			// Check if this library has already been loaded
			if (loaded_library_paths_.find(library_path) == loaded_library_paths_.end())
			{
				encoder_libraries_.emplace_back(library_path);
				loaded_library_paths_.insert(library_path);
			}
		}
	}
}

RegisterEncoder::RegisterEncoder(char const *name, EncoderCreateFunc create_func)
{
	EncoderFactory::GetInstance().RegisterEncoder(name, create_func);
}


static Encoder *h264_codec_select(VideoOptions *options, const StreamInfo &info)
{
	auto &factory = EncoderFactory::GetInstance();

	if (options->GetPlatform() == Platform::VC4)
		return factory.CreateEncoder("h264")(options, info);

	if (factory.HasEncoder("libav"))
	{
		// No hardware codec available, use x264 through libav.
		options->Set().libav_video_codec = "libx264";
		return factory.CreateEncoder("libav")(options, info);
	}

	throw std::runtime_error("Unable to find an appropriate H.264 codec");
}

static Encoder *libav_codec_select(VideoOptions *options, const StreamInfo &info)
{
	auto &factory = EncoderFactory::GetInstance();

	if (options->Get().libav_video_codec == "h264_v4l2m2m")
	{
		if (options->GetPlatform() == Platform::VC4)
			return factory.CreateEncoder("libav")(options, info);
		// No h264_v4l2m2m libav codec available, use libx264 if nothing else is provided.
		options->Set().libav_video_codec = "libx264";
	}
	return factory.CreateEncoder("libav")(options, info);
}

Encoder *Encoder::Create(VideoOptions *options, const StreamInfo &info)
{
	auto &factory = EncoderFactory::GetInstance();
	factory.LoadEncoderLibraries(options->Get().encoder_libs);

	if (strcasecmp(options->Get().codec.c_str(), "yuv420") == 0)
		return factory.CreateEncoder("null")(options, info);
	else if (strcasecmp(options->Get().codec.c_str(), "h264") == 0)
		return h264_codec_select(options, info);
	else if (factory.HasEncoder("libav") && strcasecmp(options->Get().codec.c_str(), "libav") == 0)
		return libav_codec_select(options, info);
	else if (strcasecmp(options->Get().codec.c_str(), "mjpeg") == 0)
		return factory.CreateEncoder("mjpeg")(options, info);
	throw std::runtime_error("Unrecognised codec " + options->Get().codec);
}
