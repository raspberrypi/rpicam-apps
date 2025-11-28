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
		options_->add_options()
			("quality,q", value<int>(&v_->quality)->default_value(93),
			 "Set the JPEG quality parameter")
			("exif,x", value<std::vector<std::string>>(&v_->exif),
			 "Add these extra EXIF tags to the output file")
			("timelapse", value<std::string>(&v_->timelapse_)->default_value("0ms"),
			 "Time interval between timelapse captures. If no units are provided default to ms.")
			("framestart", value<uint32_t>(&v_->framestart)->default_value(0),
			 "Initial frame counter value for timelapse captures")
			("datetime", value<bool>(&v_->datetime)->default_value(false)->implicit_value(true),
			 "Use date format for output file names")
			("timestamp", value<bool>(&v_->timestamp)->default_value(false)->implicit_value(true),
			 "Use system timestamps for output file names")
			("restart", value<unsigned int>(&v_->restart)->default_value(0),
			 "Set JPEG restart interval")
			("keypress,k", value<bool>(&v_->keypress)->default_value(false)->implicit_value(true),
			 "Perform capture when ENTER pressed")
			("signal,s", value<bool>(&v_->signal)->default_value(false)->implicit_value(true),
			 "Perform capture when signal received")
			("thumb", value<std::string>(&v_->thumb)->default_value("320:240:70"),
			 "Set thumbnail parameters as width:height:quality, or none")
			("encoding,e", value<std::string>(&v_->encoding)->default_value("jpg"),
			 "Set the desired output encoding, either jpg, png, rgb/rgb24, rgb48, bmp or yuv420")
			("raw,r", value<bool>(&v_->raw)->default_value(false)->implicit_value(true),
			 "Also save raw file in DNG format")
			("latest", value<std::string>(&v_->latest),
			 "Create a symbolic link with this name to most recent saved file")
			("immediate", value<bool>(&v_->immediate)->default_value(false)->implicit_value(true),
			 "Perform first capture immediately, with no preview phase")
			("autofocus-on-capture", value<bool>(&v_->af_on_capture)->default_value(false)->implicit_value(true),
			 "Switch to AfModeAuto and trigger a scan just before capturing a still")
			("zsl", value<bool>(&v_->zsl)->default_value(false)->implicit_value(true),
			 "Use the capture mode for preview in order to reduce the shutter lag for the final capture")
			;
		// clang-format on
	}

	virtual bool Parse(int argc, char *argv[]) override
	{
		if (Options::Parse(argc, argv) == false)
			return false;

		return v_->ParseStill();
	}

	virtual void Print() const override
	{
		Options::Print();
		v_->PrintStill();
	}
};
