/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.hpp - common program options
 */

#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>

#include <boost/program_options.hpp>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/transform.h>

#include "core/logging.hpp"
#include "core/version.hpp"

static constexpr double DEFAULT_FRAMERATE = 30.0;

struct Mode
{
	Mode() : Mode(0, 0, 0, true) {}
	Mode(unsigned int w, unsigned int h, unsigned int b, bool p) : width(w), height(h), bit_depth(b), packed(p), framerate(0) {}
	Mode(std::string const &mode_string);
	unsigned int width;
	unsigned int height;
	unsigned int bit_depth;
	bool packed;
	double framerate;
	libcamera::Size Size() const { return libcamera::Size(width, height); }
	std::string ToString() const;
	void update(const libcamera::Size &size, const std::optional<float> &fps);
};

template <typename DEFAULT>
struct TimeVal
{
	TimeVal() : value(0) {}

	void set(const std::string &s)
	{
		static const std::map<std::string, std::chrono::nanoseconds> match
		{
			{ "min", std::chrono::minutes(1) },
			{ "sec", std::chrono::seconds(1) },
			{ "s", std::chrono::seconds(1) },
			{ "ms", std::chrono::milliseconds(1) },
			{ "us", std::chrono::microseconds(1) },
			{ "ns", std::chrono::nanoseconds(1) },
		};

		try
		{
			std::size_t end_pos;
			float f = std::stof(s, &end_pos);
			value = std::chrono::duration_cast<std::chrono::nanoseconds>(f * DEFAULT { 1 });

			for (const auto &m : match)
			{
				auto found = s.find(m.first, end_pos);
				if (found != end_pos || found + m.first.length() != s.length())
					continue;
				value = std::chrono::duration_cast<std::chrono::nanoseconds>(f * m.second);
				break;
			}
		}
		catch (std::exception const &e)
		{
			throw std::runtime_error("Invalid time string provided");
		}
	}

	template <typename C = DEFAULT>
	int64_t get() const
	{
		return std::chrono::duration_cast<C>(value).count();
	}

	explicit constexpr operator bool() const
	{
		return !!value.count();
	}

	std::chrono::nanoseconds value;
};

struct Options
{
	Options()
		: set_default_lens_position(false), af_on_capture(false), options_("Valid options are", 120, 80), app_(nullptr)
	{
		using namespace boost::program_options;
		// clang-format off
		options_.add_options()
			("help,h", value<bool>(&help)->default_value(false)->implicit_value(true),
			 "Print this help message")
			("version", value<bool>(&version)->default_value(false)->implicit_value(true),
			 "Displays the build version number")
			("list-cameras", value<bool>(&list_cameras)->default_value(false)->implicit_value(true),
			 "Lists the available cameras attached to the system.")
			("camera", value<unsigned int>(&camera)->default_value(0),
			 "Chooses the camera to use. To list the available indexes, use the --list-cameras option.")
			("verbose,v", value<unsigned int>(&verbose)->default_value(1)->implicit_value(2),
			 "Set verbosity level. Level 0 is no output, 1 is default, 2 is verbose.")
			("config,c", value<std::string>(&config_file)->implicit_value("config.txt"),
			 "Read the options from a file. If no filename is specified, default to config.txt. "
			 "In case of duplicate options, the ones provided on the command line will be used. "
			 "Note that the config file must only contain the long form options.")
			("info-text", value<std::string>(&info_text)->default_value("#%frame (%fps fps) exp %exp ag %ag dg %dg"),
			 "Sets the information string on the titlebar. Available values:\n"
			 "%frame (frame number)\n%fps (framerate)\n%exp (shutter speed)\n%ag (analogue gain)"
			 "\n%dg (digital gain)\n%rg (red colour gain)\n%bg (blue colour gain)"
			 "\n%focus (focus FoM value)\n%aelock (AE locked status)"
			 "\n%lp (lens position, if known)\n%afstate (AF state, if supported)")
			("width", value<unsigned int>(&width)->default_value(0),
			 "Set the output image width (0 = use default value)")
			("height", value<unsigned int>(&height)->default_value(0),
			 "Set the output image height (0 = use default value)")
			("timeout,t", value<std::string>(&timeout_)->default_value("5sec"),
			 "Time for which program runs. If no units are provided default to ms.")
			("output,o", value<std::string>(&output),
			 "Set the output file name")
			("post-process-file", value<std::string>(&post_process_file),
			 "Set the file name for configuring the post-processing")
			("nopreview,n", value<bool>(&nopreview)->default_value(false)->implicit_value(true),
			 "Do not show a preview window")
			("preview,p", value<std::string>(&preview)->default_value("0,0,0,0"),
			 "Set the preview window dimensions, given as x,y,width,height e.g. 0,0,640,480")
			("fullscreen,f", value<bool>(&fullscreen)->default_value(false)->implicit_value(true),
			 "Use a fullscreen preview window")
			("qt-preview", value<bool>(&qt_preview)->default_value(false)->implicit_value(true),
			 "Use Qt-based preview window (WARNING: causes heavy CPU load, fullscreen not supported)")
			("hflip", value<bool>(&hflip_)->default_value(false)->implicit_value(true), "Request a horizontal flip transform")
			("vflip", value<bool>(&vflip_)->default_value(false)->implicit_value(true), "Request a vertical flip transform")
			("rotation", value<int>(&rotation_)->default_value(0), "Request an image rotation, 0 or 180")
			("roi", value<std::string>(&roi)->default_value("0,0,0,0"), "Set region of interest (digital zoom) e.g. 0.25,0.25,0.5,0.5")
			("shutter", value<std::string>(&shutter_)->default_value("0"),
			 "Set a fixed shutter speed. If no units are provided default to us")
			("analoggain", value<float>(&gain)->default_value(0),
			 "Set a fixed gain value (synonym for 'gain' option)")
			("gain", value<float>(&gain),
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
			("framerate", value<float>(&framerate_)->default_value(-1.0),
			 "Set the fixed framerate for preview and video modes")
			("denoise", value<std::string>(&denoise)->default_value("auto"),
			 "Sets the Denoise operating mode: auto, off, cdn_off, cdn_fast, cdn_hq")
			("viewfinder-width", value<unsigned int>(&viewfinder_width)->default_value(0),
			 "Width of viewfinder frames from the camera (distinct from the preview window size")
			("viewfinder-height", value<unsigned int>(&viewfinder_height)->default_value(0),
			 "Height of viewfinder frames from the camera (distinct from the preview window size)")
			("tuning-file", value<std::string>(&tuning_file)->default_value("-"),
			 "Name of camera tuning file to use, omit this option for libcamera default behaviour")
			("lores-width", value<unsigned int>(&lores_width)->default_value(0),
			 "Width of low resolution frames (use 0 to omit low resolution stream")
			("lores-height", value<unsigned int>(&lores_height)->default_value(0),
			 "Height of low resolution frames (use 0 to omit low resolution stream")
			("mode", value<std::string>(&mode_string),
			 "Camera mode as W:H:bit-depth:packing, where packing is P (packed) or U (unpacked)")
			("viewfinder-mode", value<std::string>(&viewfinder_mode_string),
			 "Camera mode for preview as W:H:bit-depth:packing, where packing is P (packed) or U (unpacked)")
			("buffer-count", value<unsigned int>(&buffer_count)->default_value(0), "Number of in-flight requests (and buffers) configured for video, raw, and still.")
			("viewfinder-buffer-count", value<unsigned int>(&viewfinder_buffer_count)->default_value(0), "Number of in-flight requests (and buffers) configured for preview window.")
			("no-raw", value<bool>(&no_raw)->default_value(false)->implicit_value(true),
			 "Disable requesting of a RAW stream. Will override any manual mode reqest the mode choice when setting framerate.")
			("autofocus-mode", value<std::string>(&afMode)->default_value("default"),
			 "Control to set the mode of the AF (autofocus) algorithm.(manual, auto, continuous)")
			("autofocus-range", value<std::string>(&afRange)->default_value("normal"),
			 "Set the range of focus distances that is scanned.(normal, macro, full)")
			("autofocus-speed", value<std::string>(&afSpeed)->default_value("normal"),
			 "Control that determines whether the AF algorithm is to move the lens as quickly as possible or more steadily.(normal, fast)")
			("autofocus-window", value<std::string>(&afWindow)->default_value("0,0,0,0"),
			"Sets AfMetering to  AfMeteringWindows an set region used, e.g. 0.25,0.25,0.5,0.5")
			("lens-position", value<std::string>(&lens_position_)->default_value(""),
			 "Set the lens to a particular focus position, expressed as a reciprocal distance (0 moves the lens to infinity), or \"default\" for the hyperfocal distance")
			("hdr", value<std::string>(&hdr)->default_value("off")->implicit_value("auto"),
			 "Enable High Dynamic Range, where supported. Available values are \"off\", \"auto\", "
			 "\"sensor\" for sensor HDR (e.g. for Camera Module 3), "
			 "\"single-exp\" for PiSP based single exposure multiframe HDR")
			("metadata", value<std::string>(&metadata),
			 "Save captured image metadata to a file or \"-\" for stdout")
			("metadata-format", value<std::string>(&metadata_format)->default_value("json"),
			 "Format to save the metadata in, either txt or json (requires --metadata)")
			("flicker-period", value<std::string>(&flicker_period_)->default_value("0s"),
			 "Manual flicker correction period"
			 "\nSet to 10000us to cancel 50Hz flicker."
			 "\nSet to 8333us to cancel 60Hz flicker.\n")
			;
		// clang-format on
	}

	virtual ~Options() {}

	bool help;
	bool version;
	bool list_cameras;
	unsigned int verbose;
	TimeVal<std::chrono::milliseconds> timeout;
	std::string config_file;
	std::string output;
	std::string post_process_file;
	unsigned int width;
	unsigned int height;
	bool nopreview;
	std::string preview;
	bool fullscreen;
	unsigned int preview_x, preview_y, preview_width, preview_height;
	libcamera::Transform transform;
	std::string roi;
	float roi_x, roi_y, roi_width, roi_height;
	TimeVal<std::chrono::microseconds> shutter;
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
	std::optional<float> framerate;
	std::string denoise;
	std::string info_text;
	unsigned int viewfinder_width;
	unsigned int viewfinder_height;
	std::string tuning_file;
	bool qt_preview;
	unsigned int lores_width;
	unsigned int lores_height;
	unsigned int camera;
	std::string mode_string;
	Mode mode;
	std::string viewfinder_mode_string;
	Mode viewfinder_mode;
	unsigned int buffer_count;
	unsigned int viewfinder_buffer_count;
	std::string afMode;
	int afMode_index;
	std::string afRange;
	int afRange_index;
	std::string afSpeed;
	int afSpeed_index;
	std::string afWindow;
	float afWindow_x, afWindow_y, afWindow_width, afWindow_height;
	std::optional<float> lens_position;
	bool set_default_lens_position;
	bool af_on_capture;
	std::string metadata;
	std::string metadata_format;
	std::string hdr;
	TimeVal<std::chrono::microseconds> flicker_period;
	bool no_raw;

	virtual bool Parse(int argc, char *argv[]);
	virtual void Print() const;

	void SetApp(RPiCamApp *app) { app_ = app; }

protected:
	boost::program_options::options_description options_;

private:
	bool hflip_;
	bool vflip_;
	int rotation_;
	float framerate_;
	std::string lens_position_;
	std::string timeout_;
	std::string shutter_;
	std::string flicker_period_;
	RPiCamApp *app_;
};
