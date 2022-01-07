/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * still_video.hpp - video capture program options
 */

#pragma once

#include <cstdio>

#include <string>

#include "options.hpp"

struct VideoOptions : public Options
{
	VideoOptions() : Options()
	{
		using namespace boost::program_options;
		// Generally we shall use zero or empty values to avoid over-writing the
		// codec's default behaviour.
		// clang-format off
		options_.add_options()
			("bitrate,b", value<uint32_t>(&bitrate)->default_value(0),
			 "Set the bitrate for encoding, in bits/second (h264 only)")
			("profile", value<std::string>(&profile),
			 "Set the encoding profile (h264 only)")
			("level", value<std::string>(&level),
			 "Set the encoding level (h264 only)")
			("intra,g", value<unsigned int>(&intra)->default_value(0),
			 "Set the intra frame period (h264 only)")
			("inline", value<bool>(&inline_headers)->default_value(false)->implicit_value(true),
			 "Force PPS/SPS header with every I frame (h264 only)")
			("codec", value<std::string>(&codec)->default_value("h264"),
			 "Set the codec to use, either h264, mjpeg or yuv420")
			("save-pts", value<std::string>(&save_pts),
			 "Save a timestamp file with this name")
			("quality,q", value<int>(&quality)->default_value(50),
			 "Set the MJPEG quality parameter (mjpeg only)")
			("listen,l", value<bool>(&listen)->default_value(false)->implicit_value(true),
			 "Listen for an incoming client network connection before sending data to the client")
			("keypress,k", value<bool>(&keypress)->default_value(false)->implicit_value(true),
			 "Pause or resume video recording when ENTER pressed")
			("signal,s", value<bool>(&signal)->default_value(false)->implicit_value(true),
			 "Pause or resume video recording when signal received")
			("initial,i", value<std::string>(&initial)->default_value("record"),
			 "Use 'pause' to pause the recording at startup, otherwise 'record' (the default)")
			("split", value<bool>(&split)->default_value(false)->implicit_value(true),
			 "Create a new output file every time recording is paused and then resumed")
			("segment", value<uint32_t>(&segment)->default_value(0),
			 "Break the recording into files of approximately this many milliseconds")
			("circular", value<size_t>(&circular)->default_value(0)->implicit_value(4),
			 "Write output to a circular buffer of the given size (in MB) which is saved on exit")
			("frames", value<unsigned int>(&frames)->default_value(0),
			 "Run for the exact number of frames specified. This will override any timeout set.")
			;
		// clang-format on
	}

	uint32_t bitrate;
	std::string profile;
	std::string level;
	unsigned int intra;
	bool inline_headers;
	std::string codec;
	std::string save_pts;
	int quality;
	bool listen;
	bool keypress;
	bool signal;
	std::string initial;
	bool pause;
	bool split;
	uint32_t segment;
	size_t circular;
	uint32_t frames;

	virtual bool Parse(int argc, char *argv[]) override
	{
		if (Options::Parse(argc, argv) == false)
			return false;

		if (width == 0)
			width = 640;
		if (height == 0)
			height = 480;
		if (strcasecmp(codec.c_str(), "h264") == 0)
			codec = "h264";
		else if (strcasecmp(codec.c_str(), "yuv420") == 0)
			codec = "yuv420";
		else if (strcasecmp(codec.c_str(), "mjpeg") == 0)
			codec = "mjpeg";
		else
			throw std::runtime_error("unrecognised codec " + codec);
		if (strcasecmp(initial.c_str(), "pause") == 0)
			pause = true;
		else if (strcasecmp(initial.c_str(), "record") == 0)
			pause = false;
		else
			throw std::runtime_error("incorrect initial value " + initial);
		if ((pause || split || segment || circular) && !inline_headers)
			std::cerr << "WARNING: consider inline headers with 'pause'/split/segment/circular" << std::endl;
		if ((split || segment) && output.find('%') == std::string::npos)
			std::cerr << "WARNING: expected % directive in output filename" << std::endl;

		return true;
	}
	virtual void Print() const override
	{
		Options::Print();
		std::cerr << "    bitrate: " << bitrate << std::endl;
		std::cerr << "    profile: " << profile << std::endl;
		std::cerr << "    level:  " << level << std::endl;
		std::cerr << "    intra: " << intra << std::endl;
		std::cerr << "    inline: " << inline_headers << std::endl;
		std::cerr << "    save-pts: " << save_pts << std::endl;
		std::cerr << "    codec: " << codec << std::endl;
		std::cerr << "    quality (for MJPEG): " << quality << std::endl;
		std::cerr << "    keypress: " << keypress << std::endl;
		std::cerr << "    signal: " << signal << std::endl;
		std::cerr << "    initial: " << initial << std::endl;
		std::cerr << "    split: " << split << std::endl;
		std::cerr << "    segment: " << segment << std::endl;
		std::cerr << "    circular: " << circular << std::endl;
	}
};
