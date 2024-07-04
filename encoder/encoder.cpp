/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * encoder.cpp - Video encoder class.
 */

#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/videodev2.h>

#include "encoder.hpp"
#include "h264_encoder.hpp"
#include "mjpeg_encoder.hpp"
#include "null_encoder.hpp"

#if LIBAV_PRESENT
#include "libav_encoder.hpp"
#endif

static Encoder *h264_codec_select(VideoOptions *options, const StreamInfo &info)
{
	if (options->GetPlatform() == Platform::VC4)
		return new H264Encoder(options, info);

#if LIBAV_PRESENT
	// No hardware codec available, use x264 through libav.
	options->libav_video_codec = "libx264";
	return new LibAvEncoder(options, info);
#endif

	throw std::runtime_error("Unable to find an appropriate H.264 codec");
}

#if LIBAV_PRESENT
static Encoder *libav_codec_select(VideoOptions *options, const StreamInfo &info)
{
	if (options->libav_video_codec == "h264_v4l2m2m")
	{
		if (options->GetPlatform() == Platform::VC4)
				return new LibAvEncoder(options, info);
		// No h264_v4l2m2m libav codec available, use libx264 if nothing else is provided.
		options->libav_video_codec = "libx264";
	}
	return new LibAvEncoder(options, info);
}
#endif

Encoder *Encoder::Create(VideoOptions *options, const StreamInfo &info)
{
	if (strcasecmp(options->codec.c_str(), "yuv420") == 0)
		return new NullEncoder(options);
	else if (strcasecmp(options->codec.c_str(), "h264") == 0)
		return h264_codec_select(options, info);
#if LIBAV_PRESENT
	else if (strcasecmp(options->codec.c_str(), "libav") == 0)
		return libav_codec_select(options, info);
#endif
	else if (strcasecmp(options->codec.c_str(), "mjpeg") == 0)
		return new MjpegEncoder(options);
	throw std::runtime_error("Unrecognised codec " + options->codec);
}
