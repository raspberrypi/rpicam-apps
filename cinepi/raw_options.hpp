/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * still_video.hpp - video capture program options
 */

#pragma once

#include <cstdio>

#include <string>

#include "core/video_options.hpp"

struct RawOptions : public VideoOptions
{   
    RawOptions() : VideoOptions()
	{
		using namespace boost::program_options;
		// Generally we shall use zero or empty values to avoid over-writing the
		// codec's default behaviour.
		// clang-format off
		options_.add_options();
	}

	uint32_t clip_number;
	std::string folder;
	int compression;
	float wb;
	std::string model;
	std::string make;
	std::string serial;
	float clipping;
};
