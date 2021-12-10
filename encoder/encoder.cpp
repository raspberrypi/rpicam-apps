/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * encoder.cpp - Video encoder class.
 */

#include <cstring>

#include "encoder.hpp"
#include "h264_encoder.hpp"
#include "mjpeg_encoder.hpp"
#include "null_encoder.hpp"

Encoder *Encoder::Create(VideoOptions const *options, const StreamInfo &info)
{
	if (strcasecmp(options->codec.c_str(), "yuv420") == 0)
		return new NullEncoder(options);
	else if (strcasecmp(options->codec.c_str(), "h264") == 0)
		return new H264Encoder(options, info);
	else if (strcasecmp(options->codec.c_str(), "mjpeg") == 0)
		return new MjpegEncoder(options);
	throw std::runtime_error("Unrecognised codec " + options->codec);
}
