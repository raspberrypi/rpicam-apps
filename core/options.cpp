/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.cpp - common program options helpers
 */
#include <fcntl.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <iomanip>
#include <iostream>

#include <libcamera/logging.h>

#include "core/options.hpp"

Mode::Mode(std::string const &mode_string)
{
	if (mode_string.empty())
		bit_depth = 0;
	else
	{
		char p;
		int n = sscanf(mode_string.c_str(), "%u:%u:%u:%c", &width, &height, &bit_depth, &p);
		if (n < 2)
			throw std::runtime_error("Invalid mode");
		else if (n == 2)
			bit_depth = 12, packed = true;
		else if (n == 3)
			packed = true;
		else if (toupper(p) == 'P')
			packed = true;
		else if (toupper(p) == 'U')
			packed = false;
		else
			throw std::runtime_error("Packing indicator should be P or U");
	}
}

std::string Mode::ToString() const
{
	if (bit_depth == 0)
		return "unspecified";
	else
	{
		std::stringstream ss;
		ss << width << ":" << height << ":" << bit_depth << ":" << (packed ? "P" : "U");
		return ss.str();
	}
}

static int xioctl(int fd, unsigned long ctl, void *arg)
{
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

bool Options::Parse(int argc, char *argv[])
{
	using namespace boost::program_options;
	using namespace libcamera;
	variables_map vm;
	// Read options from the command line
	store(parse_command_line(argc, argv, options_), vm);
	notify(vm);
	// Read options from a file if specified
	std::ifstream ifs(config_file.c_str());
	if (ifs)
	{
		store(parse_config_file(ifs, options_), vm);
		notify(vm);
	}

	// This is to get round the fact that the boost option parser does not
	// allow std::optional types.
	if (framerate_ != -1.0)
		framerate = framerate_;

	// Check if --nopreview is set, and if no info-text string was provided
	// null the defaulted string so nothing gets displayed to stderr.
	if (nopreview && vm["info-text"].defaulted())
		info_text = "";

	// lens_position is even more awkward, because we have two "default"
	// behaviours: Either no lens movement at all (if option is not given),
	// or libcamera's default control value (typically the hyperfocal).
	float f = 0.0;
	if (std::istringstream(lens_position_) >> f)
		lens_position = f;
	else if (lens_position_ == "default")
		set_default_lens_position = true;
	else if (!lens_position_.empty())
		throw std::runtime_error("Invalid lens position: " + lens_position_);

	// HDR control. Set this before opening or listing any cameras.
	// Currently this does not exist in libcamera, so go directly to V4L2
	// XXX it's not obvious which v4l2-subdev to use for which camera!
	{
		bool ok = false;
		for (int i = 0; i < 4 && !ok; i++)
		{
			std::string dev("/dev/v4l-subdev");
			dev += (char)('0' + i);
			int fd = open(dev.c_str(), O_RDWR, 0);
			if (fd < 0)
				continue;

			v4l2_control ctrl { V4L2_CID_WIDE_DYNAMIC_RANGE, hdr };
			ok = !xioctl(fd, VIDIOC_S_CTRL, &ctrl);
			close(fd);
		}
		if (hdr && !ok)
			LOG_ERROR("WARNING: Unable to set HDR mode");
	}

	// We have to pass the tuning file name through an environment variable.
	// Note that we only overwrite the variable if the option was given.
	if (tuning_file != "-")
		setenv("LIBCAMERA_RPI_TUNING_FILE", tuning_file.c_str(), 1);

	// Set the verbosity
	LibcameraApp::verbosity = verbose;

	if (verbose == 0)
		libcamera::logSetTarget(libcamera::LoggingTargetNone);

	if (help)
	{
		std::cout << options_;
		return false;
	}

	if (version)
	{
		std::cout << "libcamera-apps build: " << LibcameraAppsVersion() << std::endl;
		std::cout << "libcamera build: " << libcamera::CameraManager::version() << std::endl;
		return false;
	}

	if (list_cameras)
	{
		// Disable any libcamera logging for this bit.
		logSetTarget(LoggingTargetNone);

		std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
		int ret = cm->start();
		if (ret)
			throw std::runtime_error("camera manager failed to start, code " + std::to_string(-ret));

		std::vector<std::shared_ptr<libcamera::Camera>> cameras = cm->cameras();
		// Do not show USB webcams as these are not supported in libcamera-apps!
		auto rem = std::remove_if(cameras.begin(), cameras.end(),
								  [](auto &cam) { return cam->id().find("/usb") != std::string::npos; });
		cameras.erase(rem, cameras.end());

		if (cameras.size() != 0)
		{
			unsigned int idx = 0;
			std::cout << "Available cameras" << std::endl
					  << "-----------------" << std::endl;
			for (auto const &cam : cameras)
			{
				cam->acquire();
				std::cout << idx++ << " : " << *cam->properties().get(libcamera::properties::Model);
				auto area = cam->properties().get(properties::PixelArrayActiveAreas);
				if (area)
					std::cout << " [" << (*area)[0].size().toString() << "]";
				std::cout << " (" << cam->id() << ")" << std::endl;

				std::unique_ptr<CameraConfiguration> config = cam->generateConfiguration({libcamera::StreamRole::Raw});
				if (!config)
					throw std::runtime_error("failed to generate capture configuration");
				const StreamFormats &formats = config->at(0).formats();

				if (!formats.pixelformats().size())
					continue;

				std::cout << "    Modes: ";
				unsigned int i = 0;
				for (const auto &pix : formats.pixelformats())
				{
					if (i++) std::cout << "           ";
					std::string mode("'" + pix.toString() + "' : ");
					std::cout << mode;
					unsigned int num = formats.sizes(pix).size();
					for (const auto &size : formats.sizes(pix))
					{
						std::cout << size.toString() << " ";

						config->at(0).size = size;
						config->at(0).pixelFormat = pix;
						config->validate();
						cam->configure(config.get());

						auto fd_ctrl = cam->controls().find(&controls::FrameDurationLimits);
						auto crop_ctrl = cam->properties().get(properties::ScalerCropMaximum);
						double fps = fd_ctrl == cam->controls().end() ? NAN : (1e6 / fd_ctrl->second.min().get<int64_t>());
						std::cout << std::fixed << std::setprecision(2) << "["
								  << fps << " fps - " << crop_ctrl->toString() << " crop" << "]";
						if (--num)
						{
							std::cout << std::endl;
							for (std::size_t s = 0; s < mode.length() + 11; std::cout << " ", s++);
						}
					}
					std::cout << std::endl;
				}

				cam->release();
			}
		}
		else
			std::cout << "No cameras available!" << std::endl;

		cameras.clear();
		cm->stop();
		return false;
	}

	if (sscanf(preview.c_str(), "%u,%u,%u,%u", &preview_x, &preview_y, &preview_width, &preview_height) != 4)
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

	if (sscanf(roi.c_str(), "%f,%f,%f,%f", &roi_x, &roi_y, &roi_width, &roi_height) != 4)
		roi_x = roi_y = roi_width = roi_height = 0; // don't set digital zoom

	if (sscanf(afWindow.c_str(), "%f,%f,%f,%f", &afWindow_x, &afWindow_y, &afWindow_width, &afWindow_height) != 4)
		afWindow_x = afWindow_y = afWindow_width = afWindow_height = 0; // don't set auto focus windows

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
			{ "long", libcamera::controls::ExposureLong },
			{ "custom", libcamera::controls::ExposureCustom } };
	if (exposure_table.count(exposure) == 0)
		throw std::runtime_error("Invalid exposure mode:" + exposure);
	exposure_index = exposure_table[exposure];

	std::map<std::string, int> afMode_table =
		{ { "default", -1 },
			{ "manual", libcamera::controls::AfModeManual },
			{ "auto", libcamera::controls::AfModeAuto },
			{ "continuous", libcamera::controls::AfModeContinuous } };
	if (afMode_table.count(afMode) == 0)
		throw std::runtime_error("Invalid AfMode:" + afMode);
	afMode_index = afMode_table[afMode];

	std::map<std::string, int> afRange_table =
		{ { "normal", libcamera::controls::AfRangeNormal },
			{ "macro", libcamera::controls::AfRangeMacro },
			{ "full", libcamera::controls::AfRangeFull } };
	if (afRange_table.count(afRange) == 0)
		throw std::runtime_error("Invalid AfRange mode:" + exposure);
	afRange_index = afRange_table[afRange];


	std::map<std::string, int> afSpeed_table =
		{ { "normal", libcamera::controls::AfSpeedNormal },
		    { "fast", libcamera::controls::AfSpeedFast } };
	if (afSpeed_table.count(afSpeed) == 0)
		throw std::runtime_error("Invalid afSpeed mode:" + afSpeed);
	afSpeed_index = afSpeed_table[afSpeed];

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

	if (strcasecmp(metadata_format.c_str(), "json") == 0)
		metadata_format = "json";
	else if (strcasecmp(metadata_format.c_str(), "txt") == 0)
		metadata_format = "txt";
	else
		throw std::runtime_error("unrecognised metadata format " + metadata_format);

	mode = Mode(mode_string);
	viewfinder_mode = Mode(viewfinder_mode_string);

	return true;
}

void Options::Print() const
{
	std::cerr << "Options:" << std::endl;
	std::cerr << "    verbose: " << verbose << std::endl;
	if (!config_file.empty())
		std::cerr << "    config file: " << config_file << std::endl;
	std::cerr << "    info_text:" << info_text << std::endl;
	std::cerr << "    timeout: " << timeout << std::endl;
	std::cerr << "    width: " << width << std::endl;
	std::cerr << "    height: " << height << std::endl;
	std::cerr << "    output: " << output << std::endl;
	std::cerr << "    post_process_file: " << post_process_file << std::endl;
	std::cerr << "    rawfull: " << rawfull << std::endl;
	if (nopreview)
		std::cerr << "    preview: none" << std::endl;
	else if (fullscreen)
		std::cerr << "    preview: fullscreen" << std::endl;
	else if (preview_width == 0 || preview_height == 0)
		std::cerr << "    preview: default" << std::endl;
	else
		std::cerr << "    preview: " << preview_x << "," << preview_y << "," << preview_width << ","
					<< preview_height << std::endl;
	std::cerr << "    qt-preview: " << qt_preview << std::endl;
	std::cerr << "    transform: " << transformToString(transform) << std::endl;
	if (roi_width == 0 || roi_height == 0)
		std::cerr << "    roi: all" << std::endl;
	else
		std::cerr << "    roi: " << roi_x << "," << roi_y << "," << roi_width << "," << roi_height << std::endl;
	if (shutter)
		std::cerr << "    shutter: " << shutter << std::endl;
	if (gain)
		std::cerr << "    gain: " << gain << std::endl;
	std::cerr << "    metering: " << metering << std::endl;
	std::cerr << "    exposure: " << exposure << std::endl;
	std::cerr << "    ev: " << ev << std::endl;
	std::cerr << "    awb: " << awb << std::endl;
	if (awb_gain_r && awb_gain_b)
		std::cerr << "    awb gains: red " << awb_gain_r << " blue " << awb_gain_b << std::endl;
	std::cerr << "    flush: " << (flush ? "true" : "false") << std::endl;
	std::cerr << "    wrap: " << wrap << std::endl;
	std::cerr << "    brightness: " << brightness << std::endl;
	std::cerr << "    contrast: " << contrast << std::endl;
	std::cerr << "    saturation: " << saturation << std::endl;
	std::cerr << "    sharpness: " << sharpness << std::endl;
	std::cerr << "    framerate: " << framerate.value_or(DEFAULT_FRAMERATE) << std::endl;
	std::cerr << "    denoise: " << denoise << std::endl;
	std::cerr << "    viewfinder-width: " << viewfinder_width << std::endl;
	std::cerr << "    viewfinder-height: " << viewfinder_height << std::endl;
	std::cerr << "    tuning-file: " << (tuning_file == "-" ? "(libcamera)" : tuning_file) << std::endl;
	std::cerr << "    lores-width: " << lores_width << std::endl;
	std::cerr << "    lores-height: " << lores_height << std::endl;
	if (afMode_index != -1)
		std::cerr << "    autofocus-mode: " << afMode << std::endl;
	if (afRange_index != -1)
		std::cerr << "    autofocus-range: " << afRange << std::endl;
	if (afSpeed_index != -1)
		std::cerr << "    autofocus-speed: " << afSpeed << std::endl;
	if (afWindow_width == 0 || afWindow_height == 0)
		std::cerr << "    autofocus-window: all" << std::endl;
	else
		std::cerr << "    autofocus-window: " << afWindow_x << "," << afWindow_y << "," << afWindow_width << ","
				  << afWindow_height << std::endl;
	if (!lens_position_.empty())
		std::cerr << "    lens-position: " << lens_position_ << std::endl;
	if (hdr)
		std::cerr << "    hdr: enabled" << hdr << std::endl;
	std::cerr << "    mode: " << mode.ToString() << std::endl;
	std::cerr << "    viewfinder-mode: " << viewfinder_mode.ToString() << std::endl;
	if (buffer_count > 0)
		std::cerr << "    buffer-count: " << buffer_count << std::endl;
	if (viewfinder_buffer_count > 0)
		std::cerr << "    viewfinder-buffer-count: " << viewfinder_buffer_count << std::endl;
	std::cerr << "    metadata: " << metadata << std::endl;
	std::cerr << "    metadata-format: " << metadata_format << std::endl;
}
