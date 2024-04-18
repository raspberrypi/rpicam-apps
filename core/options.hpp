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

enum class Platform
{
	MISSING,
	UNKNOWN,
	LEGACY,
	VC4,
	PISP,
};

struct Options
{
	Options();
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
	Platform GetPlatform() const { return platform_; };

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
	Platform platform_ = Platform::UNKNOWN;
};
