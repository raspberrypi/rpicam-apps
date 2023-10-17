/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * still_options.hpp - still capture program options
 */

#pragma once

#include <cstdio>

#include "options.hpp"

struct StillOptions : public Options
{
	StillOptions() : Options()
	{
		using namespace boost::program_options;
		// clang-format off
		options_.add_options()
			("quality,q", value<int>(&quality)->default_value(93),
			 "Set the JPEG quality parameter")
			("exif,x", value<std::vector<std::string>>(&exif),
			 "Add these extra EXIF tags to the output file")
			("timelapse", value<std::string>(&timelapse_)->default_value("0ms"),
			 "Time interval between timelapse captures. If no units are provided default to ms.")
			("framestart", value<uint32_t>(&framestart)->default_value(0),
			 "Initial frame counter value for timelapse captures")
			("datetime", value<bool>(&datetime)->default_value(false)->implicit_value(true),
			 "Use date format for output file names")
			("timestamp", value<bool>(&timestamp)->default_value(false)->implicit_value(true),
			 "Use system timestamps for output file names")
			("restart", value<unsigned int>(&restart)->default_value(0),
			 "Set JPEG restart interval")
			("keypress,k", value<bool>(&keypress)->default_value(false)->implicit_value(true),
			 "Perform capture when ENTER pressed")
			("signal,s", value<bool>(&signal)->default_value(false)->implicit_value(true),
			 "Perform capture when signal received")
			("thumb", value<std::string>(&thumb)->default_value("320:240:70"),
			 "Set thumbnail parameters as width:height:quality, or none")
			("encoding,e", value<std::string>(&encoding)->default_value("jpg"),
			 "Set the desired output encoding, either jpg, png, rgb, bmp or yuv420")
			("raw,r", value<bool>(&raw)->default_value(false)->implicit_value(true),
			 "Also save raw file in DNG format")
			("latest", value<std::string>(&latest),
			 "Create a symbolic link with this name to most recent saved file")
			("immediate", value<bool>(&immediate)->default_value(false)->implicit_value(true),
			 "Perform first capture immediately, with no preview phase")
			("autofocus-on-capture", value<bool>(&af_on_capture)->default_value(false)->implicit_value(true),
			 "Switch to AfModeAuto and trigger a scan just before capturing a still")
			("zsl", value<bool>(&zsl)->default_value(false)->implicit_value(true),
			 "Switch to AfModeAuto and trigger a scan just before capturing a still")
			;
		// clang-format on
	}

	int quality;
	std::vector<std::string> exif;
	TimeVal<std::chrono::milliseconds> timelapse;
	uint32_t framestart;
	bool datetime;
	bool timestamp;
	unsigned int restart;
	bool keypress;
	bool signal;
	std::string thumb;
	unsigned int thumb_width, thumb_height, thumb_quality;
	std::string encoding;
	bool raw;
	std::string latest;
	bool immediate;
	bool zsl;

	virtual bool Parse(int argc, char *argv[]) override
	{
		if (Options::Parse(argc, argv) == false)
			return false;

		timelapse.set(timelapse_);

		if ((keypress || signal) && timelapse)
			throw std::runtime_error("keypress/signal and timelapse options are mutually exclusive");
		if (strcasecmp(thumb.c_str(), "none") == 0)
			thumb_quality = 0;
		else if (sscanf(thumb.c_str(), "%u:%u:%u", &thumb_width, &thumb_height, &thumb_quality) != 3)
			throw std::runtime_error("bad thumbnail parameters " + thumb);
		if (strcasecmp(encoding.c_str(), "jpg") == 0)
			encoding = "jpg";
		else if (strcasecmp(encoding.c_str(), "yuv420") == 0)
			encoding = "yuv420";
		else if (strcasecmp(encoding.c_str(), "rgb") == 0)
			encoding = "rgb";
		else if (strcasecmp(encoding.c_str(), "png") == 0)
			encoding = "png";
		else if (strcasecmp(encoding.c_str(), "bmp") == 0)
			encoding = "bmp";
		else
			throw std::runtime_error("invalid encoding format " + encoding);
		return true;
	}
	virtual void Print() const override
	{
		Options::Print();
		std::cerr << "    encoding: " << encoding << std::endl;
		std::cerr << "    quality: " << quality << std::endl;
		std::cerr << "    raw: " << raw << std::endl;
		std::cerr << "    restart: " << restart << std::endl;
		std::cerr << "    timelapse: " << timelapse.get() << "ms" << std::endl;
		std::cerr << "    framestart: " << framestart << std::endl;
		std::cerr << "    datetime: " << datetime << std::endl;
		std::cerr << "    timestamp: " << timestamp << std::endl;
		std::cerr << "    keypress: " << keypress << std::endl;
		std::cerr << "    signal: " << signal << std::endl;
		std::cerr << "    thumbnail width: " << thumb_width << std::endl;
		std::cerr << "    thumbnail height: " << thumb_height << std::endl;
		std::cerr << "    thumbnail quality: " << thumb_quality << std::endl;
		std::cerr << "    latest: " << latest << std::endl;
		std::cerr << "    immediate " << immediate << std::endl;
		std::cerr << "    AF on capture: " << af_on_capture << std::endl;
		std::cerr << "    Zero shutter lag: " << zsl << std::endl;
		for (auto &s : exif)
			std::cerr << "    EXIF: " << s << std::endl;
	}

private:
	std::string timelapse_;
};
