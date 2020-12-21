/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.hpp - common program options
 */

#pragma once

#include <iostream>

#include <boost/program_options.hpp>

#include <libcamera/control_ids.h>
#include <libcamera/transform.h>

struct Options
{
	Options() : options_("Valid options are")
	{
		using namespace boost::program_options;
		options_.add_options()
			("help,h", value<bool>(&help)->default_value(false)->implicit_value(true),
			 "Print this help message")
			("verbose,v", value<bool>(&verbose)->default_value(false)->implicit_value(true),
			 "Output extra debug and diagnostics")
			("width", value<unsigned int>(&width)->default_value(0),
			 "Set the output image width (0 = use default value)")
			("height", value<unsigned int>(&height)->default_value(0),
			 "Set the output image height (0 = use default value)")
			("timeout,t", value<uint64_t>(&timeout)->default_value(5000),
			 "Time (in ms) for which program runs")
			("output,o", value<std::string>(&output),
			 "Set the output file name")
			("rawfull", value<bool>(&rawfull)->default_value(false)->implicit_value(true),
			 "Force use of full resolution raw frames")
			("nopreview,n", value<bool>(&nopreview)->default_value(false)->implicit_value(true),
			 "Do not show a preview window")
			("preview,p", value<std::string>(&preview)->default_value("0,0,0,0"),
			 "Set the preview window dimensions, given as x,y,width,height e.g. 0,0,640,480")
			("fullscreen,f", value<bool>(&fullscreen)->default_value(false)->implicit_value(true),
			 "Use a fullscreen preview window")
			("hflip", value<bool>(&hflip_)->default_value(false)->implicit_value(true), "Request a horizontal flip transform")
			("vflip", value<bool>(&vflip_)->default_value(false)->implicit_value(true), "Request a vertical flip transform")
			("rotation", value<int>(&rotation_)->default_value(0), "Request an image rotation, 0 or 180")
			("roi", value<std::string>(&roi)->default_value("0,0,0,0"), "Set region of interest (digital zoom) e.g. 0.25,0.25,0.5,0.5")
			("shutter", value<float>(&shutter)->default_value(0),
			 "Set a fixed shutter speed")
			("analoggain", value<float>(&gain)->default_value(0),
			 "Set a fixed gain value (synonym for 'gain' option)")
			("gain", value<float>(&gain)->default_value(0),
			 "Set a fixed gain value")
			("metering", value<std::string>(&metering)->default_value("centre"),
			 "Set the metering mode (centre, spot, average, custom)")
			("exposure", value<std::string>(&exposure)->default_value("normal"),
			 "Set the exposure mode (normal, sport)")
			("ev", value<float>(&ev)->default_value(0),
			 "Set the EV exposure compensation, where 0 = no change")
			("awb", value<std::string>(&awb)->default_value("auto"),
			 "Set the AWB mode (auto, incandescent, tungsten, fluorescent, indoor, daylight, cloudy, custom)")
			("awbgains", value<std::string>(&awbgains)->default_value("0,0"),
			 "Set explict red and blue gains (disable the automatic AWB algorithm)")
			("flush", value<bool>(&flush)->default_value(false)->implicit_value(true),
			 "Flush output data as soon as possible")
			("wrap", value<unsigned int>(&wrap)->default_value(0),
			 "When writing multiple output files, reset the counter when it reaches this number")
			("brightness", value<float>(&brightness)->default_value(0),
			 "Adjust the brightness of the output images, in the range -1.0 to 1.0")
			("contrast", value<float>(&contrast)->default_value(1.0),
			 "Adjust the contrast of the output image, where 1.0 = normal contrast")
			("saturation", value<float>(&saturation)->default_value(1.0),
			 "Adjust the colour saturation of the output, where 1.0 = normal and 0.0 = greyscale")
			("sharpness", value<float>(&sharpness)->default_value(1.0),
			 "Adjust the sharpness of the output image, where 1.0 = normal sharpening")
			("framerate", value<float>(&framerate)->default_value(30.0),
			 "Set the fixed framerate for preview and video modes")
			;
	}

	bool help;
	bool verbose;
	uint64_t timeout; // in ms
	std::string output;
	unsigned int width;
	unsigned int height;
	bool rawfull;
	bool nopreview;
	std::string preview;
	bool fullscreen;
	unsigned int preview_x, preview_y, preview_width, preview_height;
	libcamera::Transform transform;
	std::string roi;
	float roi_x, roi_y, roi_width, roi_height;
	float shutter;
	float gain;
	std::string metering;
	int metering_index;
	std::string exposure;
	int exposure_index;
	float ev;
	std::string awb;
	int awb_index;
	std::string awbgains;
	float awb_gain_r;
	float awb_gain_b;
	bool flush;
	unsigned int wrap;
	float brightness;
	float contrast;
	float saturation;
	float sharpness;
	float framerate;

	virtual bool Parse(int argc, char *argv[])
	{
		using namespace boost::program_options;
		using namespace libcamera;
		variables_map vm;
		store(parse_command_line(argc, argv, options_), vm);
		notify(vm);
		if (help)
		{
			std::cout << options_;
			return false;
		}

		if (sscanf(preview.c_str(), "%u,%u,%u,%u",
				   &preview_x, &preview_y, &preview_width, &preview_height) != 4)
			preview_x = preview_y = preview_width = preview_height = 0; // use default window

		transform = Transform::Identity;
		if (hflip_)
			transform = Transform::HFlip * transform;
		if (vflip_)
			transform = Transform::VFlip * transform;
		bool ok;
		Transform rot = transformFromRotation(rotation_, &ok);
		if (!ok)
			throw std::runtime_error("illegal rotation value");
		transform = rot * transform;
		if (!!(transform & Transform::Transpose))
			throw std::runtime_error("transforms requiring transpose not supported");

		if (sscanf(roi.c_str(), "%f,%f,%f,%f",
				   &roi_x, &roi_y, &roi_width, &roi_height) != 4)
			roi_x = roi_y = roi_width = roi_height = 0; // don't set digital zoom

		std::map<std::string, int> metering_table =
			{ { "centre", libcamera::controls::MeteringCentreWeighted },
			  { "spot", libcamera::controls::MeteringSpot },
			  { "average", libcamera::controls::MeteringMatrix },
			  { "matrix", libcamera::controls::MeteringMatrix },
			  { "custom", libcamera::controls::MeteringCustom } };
		if (metering_table.count(metering) == 0)
			throw std::runtime_error("Invalid metering mode: " + metering);
		metering_index = metering_table[metering];

		std::map<std::string, int> exposure_table =
			{ { "normal", libcamera::controls::ExposureNormal },
			  { "sport", libcamera::controls::ExposureShort },
			  { "short", libcamera::controls::ExposureShort },
			  // long mode?
			  { "custom", libcamera::controls::ExposureCustom } };
		if (exposure_table.count(exposure) == 0)
			throw std::runtime_error("Invalid exposure mode:" + exposure);
		exposure_index = exposure_table[exposure];

		std::map<std::string, int> awb_table =
			{ { "auto", libcamera::controls::AwbAuto },
			  { "normal", libcamera::controls::AwbAuto },
			  { "incandescent", libcamera::controls::AwbIncandescent },
			  { "tungsten", libcamera::controls::AwbTungsten },
			  { "fluorescent", libcamera::controls::AwbFluorescent },
			  { "indoor", libcamera::controls::AwbIndoor },
			  { "daylight", libcamera::controls::AwbDaylight },
			  { "cloudy", libcamera::controls::AwbCloudy },
			  { "custom", libcamera::controls::AwbCustom } };
		if (awb_table.count(awb) == 0)
			throw std::runtime_error("Invalid AWB mode: " + awb);
		awb_index = awb_table[awb];

		if (sscanf(awbgains.c_str(), "%f,%f", &awb_gain_r, &awb_gain_b) != 2)
			throw std::runtime_error("Invalid AWB gains");

		brightness = std::clamp(brightness, -1.0f, 1.0f);
		contrast = std::clamp(contrast, 0.0f, 15.99f); // limits are arbitrary..
		saturation = std::clamp(saturation, 0.0f, 15.99f); // limits are arbitrary..
		sharpness = std::clamp(sharpness, 0.0f, 15.99f); // limits are arbitrary..

		return true;
	}
	virtual void Print() const
	{
		std::cout << "Options:" << std::endl;
		std::cout << "    verbose: " << verbose << std::endl;
		std::cout << "    timeout: " << timeout << std::endl;
		std::cout << "    width: " << width << std::endl;
		std::cout << "    height: " << height << std::endl;
		std::cout << "    output: " << output << std::endl;
		std::cout << "    rawfull: " << rawfull << std::endl;
		if (nopreview)
			std::cout << "    preview: none" << std::endl;
		else if (fullscreen)
			std::cout << "    preview: fullscreen" << std::endl;
		else if (preview_width == 0 || preview_height == 0)
			std::cout << "    preview: default" << std::endl;
		else
			std::cout << "    preview: " << preview_x << "," << preview_y << ","
					  << preview_width << "," << preview_height << std::endl;
		std::cout << "    transform: " << transformToString(transform) << std::endl;
		if (roi_width == 0 || roi_height == 0)
			std::cout << "    roi: all" << std::endl;
		else
			std::cout << "    roi: " << roi_x << "," << roi_y << ","
					  << roi_width << "," << roi_height << std::endl;
		if (shutter)
			std::cout << "    shutter: " << shutter << std::endl;
		if (gain)
			std::cout << "    gain: " << gain << std::endl;
		std::cout << "    metering: " << metering << std::endl;
		std::cout << "    exposure: " << exposure << std::endl;
		std::cout << "    ev: " << ev << std::endl;
		std::cout << "    awb: " << awb << std::endl;
		if (awb_gain_r && awb_gain_b)
			std::cout << "    awb gains: red " << awb_gain_r << " blue " << awb_gain_b << std::endl;
		std::cout << "    flush: " << (flush ? "true" : "false") << std::endl;
		std::cout << "    wrap: " << wrap << std::endl;
		std::cout << "    brightness: " << brightness << std::endl;
		std::cout << "    contrast: " << contrast << std::endl;
		std::cout << "    saturation: " << saturation << std::endl;
		std::cout << "    sharpness: " << sharpness << std::endl;
		std::cout << "    framerate: " << framerate << std::endl;
	}

protected:
	boost::program_options::options_description options_;

private:
	bool hflip_;
	bool vflip_;
	int rotation_;
};
