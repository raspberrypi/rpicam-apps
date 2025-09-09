/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.cpp - common program options helpers
 */
#include <algorithm>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <map>
#include <string>
#include <sys/ioctl.h>

#include <libcamera/formats.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>

#include "core/options.hpp"

namespace fs = std::filesystem;

namespace
{

const std::map<int, std::string> &cfa_map()
{
	static const std::map<int, std::string> map {
		{ properties::draft::ColorFilterArrangementEnum::RGGB, "RGGB" },
		{ properties::draft::ColorFilterArrangementEnum::GRBG, "GRBG" },
		{ properties::draft::ColorFilterArrangementEnum::GBRG, "GBRG" },
		{ properties::draft::ColorFilterArrangementEnum::RGB, "RGB" },
		{ properties::draft::ColorFilterArrangementEnum::MONO, "MONO" },
	};

	return map;
}

const std::map<libcamera::PixelFormat, unsigned int> &bayer_formats()
{
	static const std::map<libcamera::PixelFormat, unsigned int> map {
		{ libcamera::formats::SRGGB10_CSI2P, 10 },
		{ libcamera::formats::SGRBG10_CSI2P, 10 },
		{ libcamera::formats::SBGGR10_CSI2P, 10 },
		{ libcamera::formats::R10_CSI2P,     10 },
		{ libcamera::formats::SGBRG10_CSI2P, 10 },
		{ libcamera::formats::SRGGB12_CSI2P, 12 },
		{ libcamera::formats::SGRBG12_CSI2P, 12 },
		{ libcamera::formats::SBGGR12_CSI2P, 12 },
		{ libcamera::formats::SGBRG12_CSI2P, 12 },
		{ libcamera::formats::SRGGB14_CSI2P, 14 },
		{ libcamera::formats::SGRBG14_CSI2P, 14 },
		{ libcamera::formats::SBGGR14_CSI2P, 14 },
		{ libcamera::formats::SGBRG14_CSI2P, 14 },
		{ libcamera::formats::SRGGB16,       16 },
		{ libcamera::formats::SGRBG16,       16 },
		{ libcamera::formats::SBGGR16,       16 },
		{ libcamera::formats::SGBRG16,       16 },
	};

	return map;
}

}

Mode::Mode(std::string const &mode_string) : Mode()
{
	if (!mode_string.empty())
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
		if (framerate)
			ss << "(" << framerate << ")";
		return ss.str();
	}
}

void Mode::update(const libcamera::Size &size, const std::optional<float> &fps)
{
	if (!width)
		width = size.width;
	if (!height)
		height = size.height;
	if (!bit_depth)
		bit_depth = 12;
	if (fps)
		framerate = fps.value();
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

static bool set_imx708_subdev_hdr_ctrl(int en, const std::string &cam_id)
{
	for (unsigned int i = 0; i < 16; i++)
	{
		const fs::path test_dir { "/sys/class/video4linux/v4l-subdev" + std::to_string(i) + "/device" };
		const fs::path module_dir { test_dir.string() + "/driver/module" };
		const fs::path id_dir { test_dir.string() + "/of_node" };

		if (fs::exists(module_dir) && fs::is_symlink(module_dir))
		{
			fs::path ln = fs::read_symlink(module_dir);
			if (ln.string().find("imx708") != std::string::npos &&
				fs::is_symlink(id_dir) && fs::read_symlink(id_dir).string().find(cam_id) != std::string::npos)
			{
				const std::string dev_node { "/dev/v4l-subdev" + std::to_string(i) };
				int fd = open(dev_node.c_str(), O_RDONLY, 0);
				if (fd < 0)
					continue;

				v4l2_control ctrl { V4L2_CID_WIDE_DYNAMIC_RANGE, en };
				if (!xioctl(fd, VIDIOC_G_CTRL, &ctrl) && ctrl.value != en)
				{
					ctrl.value = en;
					if (!xioctl(fd, VIDIOC_S_CTRL, &ctrl))
					{
						close(fd);
						return true;
					}
				}
				close(fd);
			}
		}
	}
	return false;
}

Platform get_platform()
{
	bool unknown = false;
	for (unsigned int device_num = 0; device_num < 256; device_num++)
	{
		char device_name[16];
		snprintf(device_name, sizeof(device_name), "/dev/video%u", device_num);
		int fd = open(device_name, O_RDWR, 0);
		if (fd < 0)
			continue;

		v4l2_capability caps;
		unsigned long request = VIDIOC_QUERYCAP;

		int ret = ioctl(fd, request, &caps);
		close(fd);

		if (ret)
			continue;

		// We are not concerned with UVC devices for now.
		if (!strncmp((char *)caps.driver, "uvcvideo", sizeof(caps.card)))
			continue;

		if (!strncmp((char *)caps.card, "bcm2835-isp", sizeof(caps.card)))
			return Platform::VC4;
		else if (!strncmp((char *)caps.card, "pispbe", sizeof(caps.card)))
			return Platform::PISP;
		else if (!strncmp((char *)caps.card, "bm2835 mmal", sizeof(caps.card)))
			return Platform::LEGACY;
		else
			unknown = true;
	}

	return unknown ? Platform::UNKNOWN : Platform::MISSING;
}

Options::Options()
	: options_(std::make_unique<boost::program_options::options_description>("Valid options are", 120, 80)),
	  app_(nullptr)
{
	using namespace boost::program_options;

	// clang-format off
	options_->add_options()
		("help,h", value<bool>(&v_->help)->default_value(false)->implicit_value(true),
			"Print this help message")
		("version", value<bool>(&v_->version)->default_value(false)->implicit_value(true),
			"Displays the build version number")
		("list-cameras", value<bool>(&v_->list_cameras)->default_value(false)->implicit_value(true),
			"Lists the available cameras attached to the system.")
		("camera", value<unsigned int>(&v_->camera)->default_value(0),
			"Chooses the camera to use. To list the available indexes, use the --list-cameras option.")
		("verbose,v", value<unsigned int>(&v_->verbose)->default_value(1)->implicit_value(2),
			"Set verbosity level. Level 0 is no output, 1 is default, 2 is verbose.")
		("config,c", value<std::string>(&v_->config_file)->implicit_value("config.txt"),
			"Read the options from a file. If no filename is specified, default to config.txt. "
			"In case of duplicate options, the ones provided on the command line will be used. "
			"Note that the config file must only contain the long form options.")
		("info-text", value<std::string>(&v_->info_text)->default_value("#%frame (%fps fps) exp %exp ag %ag dg %dg"),
			"Sets the information string on the titlebar. Available values:\n"
			"%frame (frame number)\n%fps (framerate)\n%exp (shutter speed)\n%ag (analogue gain)"
			"\n%dg (digital gain)\n%rg (red colour gain)\n%bg (blue colour gain)"
			"\n%focus (focus FoM value)\n%aelock (AE locked status)"
			"\n%lp (lens position, if known)\n%temp (sensor temperature, if available)"
			"\n%afstate (AF state, if supported)")
		("width", value<unsigned int>(&v_->width)->default_value(0),
			"Set the output image width (0 = use default value)")
		("height", value<unsigned int>(&v_->height)->default_value(0),
			"Set the output image height (0 = use default value)")
		("timeout,t", value<std::string>(&v_->timeout_)->default_value("5sec"),
			"Time for which program runs. If no units are provided default to ms.")
		("output,o", value<std::string>(&v_->output),
			"Set the output file name")
		("post-process-file", value<std::string>(&v_->post_process_file),
			"Set the file name for configuring the post-processing")
		("post-process-libs", value<std::string>(&v_->post_process_libs),
			"Set a custom location for the post-processing library .so files")
		("nopreview,n", value<bool>(&v_->nopreview)->default_value(false)->implicit_value(true),
			"Do not show a preview window")
		("preview,p", value<std::string>(&v_->preview)->default_value("0,0,0,0"),
			"Set the preview window dimensions, given as x,y,width,height e.g. 0,0,640,480")
		("fullscreen,f", value<bool>(&v_->fullscreen)->default_value(false)->implicit_value(true),
			"Use a fullscreen preview window")
		("qt-preview", value<bool>(&v_->qt_preview)->default_value(false)->implicit_value(true),
			"Use Qt-based preview window (WARNING: causes heavy CPU load, fullscreen not supported)")
		("preview-libs", value<std::string>(&v_->preview_libs)->default_value(""),
			"Set a custom location for the preview library .so files")
		("hflip", value<bool>(&v_->hflip_)->default_value(false)->implicit_value(true), "Request a horizontal flip transform")
		("vflip", value<bool>(&v_->vflip_)->default_value(false)->implicit_value(true), "Request a vertical flip transform")
		("rotation", value<int>(&v_->rotation_)->default_value(0), "Request an image rotation, 0 or 180")
		("roi", value<std::string>(&v_->roi)->default_value("0,0,0,0"), "Set region of interest (digital zoom) e.g. 0.25,0.25,0.5,0.5")
		("shutter", value<std::string>(&v_->shutter_)->default_value("0"),
			"Set a fixed shutter speed. If no units are provided default to us")
		("analoggain", value<float>(&v_->gain)->default_value(0),
			"Set a fixed gain value (synonym for 'gain' option)")
		("gain", value<float>(&v_->gain),
			"Set a fixed gain value")
		("metering", value<std::string>(&v_->metering)->default_value("centre"),
			"Set the metering mode (centre, spot, average, custom)")
		("exposure", value<std::string>(&v_->exposure)->default_value("normal"),
			"Set the exposure mode (normal, sport)")
		("ev", value<float>(&v_->ev)->default_value(0),
			"Set the EV exposure compensation, where 0 = no change")
		("awb", value<std::string>(&v_->awb)->default_value("auto"),
			"Set the AWB mode (auto, incandescent, tungsten, fluorescent, indoor, daylight, cloudy, custom)")
		("awbgains", value<std::string>(&v_->awbgains)->default_value("0,0"),
			"Set explict red and blue gains (disable the automatic AWB algorithm)")
		("ccm", value<std::string>(&v_->ccm)->default_value(""),
			"Set an explicit colour correction matrix (NOTE: must also set explicit AWB gains)")
		("flush", value<bool>(&v_->flush)->default_value(false)->implicit_value(true),
			"Flush output data as soon as possible")
		("wrap", value<unsigned int>(&v_->wrap)->default_value(0),
			"When writing multiple output files, reset the counter when it reaches this number")
		("brightness", value<float>(&v_->brightness)->default_value(0),
			"Adjust the brightness of the output images, in the range -1.0 to 1.0")
		("contrast", value<float>(&v_->contrast)->default_value(1.0),
			"Adjust the contrast of the output image, where 1.0 = normal contrast")
		("saturation", value<float>(&v_->saturation)->default_value(1.0),
			"Adjust the colour saturation of the output, where 1.0 = normal and 0.0 = greyscale")
		("sharpness", value<float>(&v_->sharpness)->default_value(1.0),
			"Adjust the sharpness of the output image, where 1.0 = normal sharpening")
		("framerate", value<float>(&v_->framerate_)->default_value(-1.0),
			"Set the fixed framerate for preview and video modes")
		("denoise", value<std::string>(&v_->denoise)->default_value("auto"),
			"Sets the Denoise operating mode: auto, off, cdn_off, cdn_fast, cdn_hq")
		("viewfinder-width", value<unsigned int>(&v_->viewfinder_width)->default_value(0),
			"Width of viewfinder frames from the camera (distinct from the preview window size")
		("viewfinder-height", value<unsigned int>(&v_->viewfinder_height)->default_value(0),
			"Height of viewfinder frames from the camera (distinct from the preview window size)")
		("tuning-file", value<std::string>(&v_->tuning_file)->default_value("-"),
			"Name of camera tuning file to use, omit this option for libcamera default behaviour")
		("lores-width", value<unsigned int>(&v_->lores_width)->default_value(0),
			"Width of low resolution frames (use 0 to omit low resolution stream)")
		("lores-height", value<unsigned int>(&v_->lores_height)->default_value(0),
			"Height of low resolution frames (use 0 to omit low resolution stream)")
		("lores-par", value<bool>(&v_->lores_par)->default_value(false)->implicit_value(true),
			"Preserve the pixel aspect ratio of the low res image (where possible) by applying a different crop on the stream.")
		("mode", value<std::string>(&v_->mode_string),
			"Camera mode as W:H:bit-depth:packing, where packing is P (packed) or U (unpacked)")
		("viewfinder-mode", value<std::string>(&v_->viewfinder_mode_string),
			"Camera mode for preview as W:H:bit-depth:packing, where packing is P (packed) or U (unpacked)")
		("buffer-count", value<unsigned int>(&v_->buffer_count)->default_value(0), "Number of in-flight requests (and buffers) configured for video, raw, and still.")
		("viewfinder-buffer-count", value<unsigned int>(&v_->viewfinder_buffer_count)->default_value(0), "Number of in-flight requests (and buffers) configured for preview window.")
		("no-raw", value<bool>(&v_->no_raw)->default_value(false)->implicit_value(true),
			"Disable requesting of a RAW stream. Will override any manual mode reqest the mode choice when setting framerate.")
		("autofocus-mode", value<std::string>(&v_->afMode)->default_value("default"),
			"Control to set the mode of the AF (autofocus) algorithm.(manual, auto, continuous)")
		("autofocus-range", value<std::string>(&v_->afRange)->default_value("normal"),
			"Set the range of focus distances that is scanned.(normal, macro, full)")
		("autofocus-speed", value<std::string>(&v_->afSpeed)->default_value("normal"),
			"Control that determines whether the AF algorithm is to move the lens as quickly as possible or more steadily.(normal, fast)")
		("autofocus-window", value<std::string>(&v_->afWindow)->default_value("0,0,0,0"),
		"Sets AfMetering to  AfMeteringWindows an set region used, e.g. 0.25,0.25,0.5,0.5")
		("lens-position", value<std::string>(&v_->lens_position_)->default_value(""),
			"Set the lens to a particular focus position, expressed as a reciprocal distance (0 moves the lens to infinity), or \"default\" for the hyperfocal distance")
		("hdr", value<std::string>(&v_->hdr)->default_value("off")->implicit_value("auto"),
			"Enable High Dynamic Range, where supported. Available values are \"off\", \"auto\", "
			"\"sensor\" for sensor HDR (e.g. for Camera Module 3), "
			"\"single-exp\" for PiSP based single exposure multiframe HDR")
		("metadata", value<std::string>(&v_->metadata),
			"Save captured image metadata to a file or \"-\" for stdout")
		("metadata-format", value<std::string>(&v_->metadata_format)->default_value("json"),
			"Format to save the metadata in, either txt or json (requires --metadata)")
		("flicker-period", value<std::string>(&v_->flicker_period_)->default_value("0s"),
			"Manual flicker correction period"
			"\nSet to 10000us to cancel 50Hz flicker."
			"\nSet to 8333us to cancel 60Hz flicker.\n")
		;
	// clang-format on

	// This is really the best place to cache the platform, all components
	// in rpicam-apps get the options structure;
	platform_ = get_platform();
}

bool Options::Parse(int argc, char *argv[])
{
	using namespace boost::program_options;
	variables_map vm;
	// Read options from the command line
	store(parse_command_line(argc, argv, *options_), vm);
	notify(vm);
	// Read options from a file if specified
	std::ifstream ifs(v_->config_file.c_str());
	if (ifs)
	{
		store(parse_config_file(ifs, *options_), vm);
		notify(vm);
	}

	if (v_->help)
	{
		std::cout << *options_;
		return false;
	}

	return v_->Parse(vm, app_);
}

bool OptsInternal::Parse(boost::program_options::variables_map &vm, RPiCamApp *app)
{
	using namespace libcamera;

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

	// Convert time strings to durations
	timeout.set(timeout_);
	shutter.set(shutter_);
	flicker_period.set(flicker_period_);

	if (version)
	{
		std::cout << "rpicam-apps build: " << RPiCamAppsVersion() << std::endl;
		std::cout << "rpicam-apps capabilites: " << RPiCamAppsCapabilities(preview_libs, encoder_libs) << std::endl;
		std::cout << "libcamera build: " << libcamera::CameraManager::version() << std::endl;
		return false;
	}

	// We have to pass the tuning file name through an environment variable.
	// Note that we only overwrite the variable if the option was given.
	if (tuning_file != "-")
		setenv("LIBCAMERA_RPI_TUNING_FILE", tuning_file.c_str(), 1);

	if (hdr != "off" && hdr != "single-exp" && hdr != "sensor" && hdr != "auto")
		throw std::runtime_error("Invalid HDR option provided: " + hdr);

	if (!verbose || list_cameras)
		libcamera::logSetTarget(libcamera::LoggingTargetNone);

	app->initCameraManager();

	bool log_env_set = getenv("LIBCAMERA_LOG_LEVELS");
	// Unconditionally set the logging level to error for a bit.
	if (!log_env_set)
		libcamera::logSetLevel("*", "ERROR");

	std::vector<std::shared_ptr<libcamera::Camera>> cameras = app->GetCameras();
	if (camera < cameras.size())
	{
		const std::string cam_id = *cameras[camera]->properties().get(libcamera::properties::Model);

		if (cam_id.find("imx708") != std::string::npos)
		{
			// HDR control. Set the sensor control before opening or listing any cameras.
			// Start by disabling HDR unconditionally. Reset the camera manager if we have
			// actually switched the value of the control
			bool changed = set_imx708_subdev_hdr_ctrl(0, cameras[camera]->id());

			if (hdr == "sensor" || hdr == "auto")
			{
				// Turn on sensor HDR.  Reset the camera manager if we have switched the value of the control.
				changed |= set_imx708_subdev_hdr_ctrl(1, cameras[camera]->id());
				hdr = "sensor";
			}

			if (changed)
			{
				cameras.clear();
				app->initCameraManager();
				cameras = app->GetCameras();
			}
		}
	}

	if (list_cameras)
	{
		RPiCamApp::verbosity = 1;

		if (cameras.size() != 0)
		{
			unsigned int idx = 0;
			std::cout << "Available cameras" << std::endl
					  << "-----------------" << std::endl;
			for (auto const &cam : cameras)
			{
				cam->acquire();

				std::stringstream sensor_props;
				sensor_props << idx++ << " : " << *cam->properties().get(libcamera::properties::Model) << " [";

				auto area = cam->properties().get(properties::PixelArrayActiveAreas);
				if (area)
					sensor_props << (*area)[0].size().toString() << " ";

				std::unique_ptr<CameraConfiguration> config =
					cam->generateConfiguration({ libcamera::StreamRole::Raw, libcamera::StreamRole::Viewfinder });
				if (!config)
					throw std::runtime_error("failed to generate capture configuration");
				const StreamFormats &formats = config->at(0).formats();

				if (!formats.pixelformats().size())
					continue;

				unsigned int bits = 0;
				for (const auto &pix : formats.pixelformats())
				{
					const auto &b = bayer_formats().find(pix);
					if (b != bayer_formats().end() && b->second > bits)
						bits = b->second;
				}
				if (bits)
					sensor_props << bits << "-bit ";

				auto cfa = cam->properties().get(properties::draft::ColorFilterArrangement);
				if (cfa && cfa_map().count(*cfa))
					sensor_props << cfa_map().at(*cfa) << " ";

				sensor_props.seekp(-1, sensor_props.cur);
				sensor_props << "] (" << cam->id() << ")";
				std::cout << sensor_props.str() << std::endl;

				ControlInfoMap control_map;
				Size max_size;
				PixelFormat max_fmt;

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
						RPiCamApp::SensorMode sensor_mode(size, pix, 0);
						std::cout << size.toString() << " ";

						config->at(0).size = size;
						config->at(0).pixelFormat = pix;
						config->sensorConfig = libcamera::SensorConfiguration();
						config->sensorConfig->outputSize = size;
						config->sensorConfig->bitDepth = sensor_mode.depth();
						config->validate();
						cam->configure(config.get());

						if (size > max_size)
						{
							control_map = cam->controls();
							max_fmt = pix;
							max_size = size;
						}

						auto fd_ctrl = cam->controls().find(&controls::FrameDurationLimits);
						auto crop_ctrl = cam->controls().at(&controls::ScalerCrop).max().get<Rectangle>();
						double fps = fd_ctrl == cam->controls().end() ? NAN : (1e6 / fd_ctrl->second.min().get<int64_t>());
						std::cout << std::fixed << std::setprecision(2) << "["
								  << fps << " fps - " << crop_ctrl.toString() << " crop" << "]";
						if (--num)
						{
							std::cout << std::endl;
							for (std::size_t s = 0; s < mode.length() + 11; std::cout << " ", s++);
						}
					}
					std::cout << std::endl;
				}

				if (verbose > 1)
				{
					std::stringstream ss;
					ss << "\n    Available controls for " << max_size.toString() << " " << max_fmt.toString()
					   << " mode:\n    ";
					std::cout << ss.str();
					for (std::size_t s = 0; s < ss.str().length() - 10; std::cout << "-", s++);
					std::cout << std::endl;

					std::vector<std::string> ctrls;
					for (auto const &[id, info] : control_map)
						ctrls.emplace_back(id->name() + " : " + info.toString());
					std::sort(ctrls.begin(), ctrls.end(), [](auto const &l, auto const &r) { return l < r; });
					for (auto const &c : ctrls)
						std::cout << "    " << c << std::endl;
				}

				std::cout << std::endl;
				cam->release();
			}
		}
		else
			std::cout << "No cameras available!" << std::endl;

		verbose = 1;
		return false;
	}

	// Reset log level to Info.
	if (verbose && !log_env_set)
		libcamera::logSetLevel("*", "INFO");

	// Set the verbosity
	RPiCamApp::verbosity = verbose;

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

	if (!ccm.empty() &&
		sscanf(ccm.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f",
			   &ccm_values[0], &ccm_values[1], &ccm_values[2],
			   &ccm_values[3], &ccm_values[4], &ccm_values[5],
			   &ccm_values[6], &ccm_values[7], &ccm_values[8]) != 9)
		throw std::runtime_error("Invalid CCM - expect 9 comma-separated floating point numbers");

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

void OptsInternal::Print() const
{
	std::cerr << "Options:" << std::endl;
	std::cerr << "    verbose: " << verbose << std::endl;
	if (!config_file.empty())
		std::cerr << "    config file: " << config_file << std::endl;
	std::cerr << "    info_text:" << info_text << std::endl;
	std::cerr << "    timeout: " << timeout.get() << "ms" << std::endl;
	std::cerr << "    width: " << width << std::endl;
	std::cerr << "    height: " << height << std::endl;
	std::cerr << "    output: " << output << std::endl;
	std::cerr << "    post_process_file: " << post_process_file << std::endl;
	std::cerr << "    post_process_libs: " << post_process_libs << std::endl;
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
		std::cerr << "    shutter: " << shutter.get() << "us" << std::endl;
	if (gain)
		std::cerr << "    gain: " << gain << std::endl;
	std::cerr << "    metering: " << metering << std::endl;
	std::cerr << "    exposure: " << exposure << std::endl;
	if (flicker_period)
		std::cerr << "    flicker period: " << flicker_period.get() << "us" << std::endl;
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
	std::cerr << "    lores-par: " << lores_par << std::endl;
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
	std::cerr << "    hdr: " << hdr << std::endl;
	std::cerr << "    mode: " << mode.ToString() << std::endl;
	std::cerr << "    viewfinder-mode: " << viewfinder_mode.ToString() << std::endl;
	if (buffer_count > 0)
		std::cerr << "    buffer-count: " << buffer_count << std::endl;
	if (viewfinder_buffer_count > 0)
		std::cerr << "    viewfinder-buffer-count: " << viewfinder_buffer_count << std::endl;
	std::cerr << "    metadata: " << metadata << std::endl;
	std::cerr << "    metadata-format: " << metadata_format << std::endl;
}

bool OptsInternal::ParseVideo()
{
	bitrate.set(bitrate_);
	av_sync.set(av_sync_);
	audio_bitrate.set(audio_bitrate_);
	if (width == 0)
		width = 640;
	if (height == 0)
		height = 480;
	if (strcasecmp(codec.c_str(), "h264") == 0)
		codec = "h264";
	else if (strcasecmp(codec.c_str(), "libav") == 0)
		codec = "libav";
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
		LOG_ERROR("WARNING: consider inline headers with 'pause'/split/segment/circular");
	if ((split || segment) && output.find('%') == std::string::npos)
		LOG_ERROR("WARNING: expected % directive in output filename");

	// From https://en.wikipedia.org/wiki/Advanced_Video_Coding#Levels
	double mbps = ((width + 15) >> 4) * ((height + 15) >> 4) * framerate.value_or(DEFAULT_FRAMERATE);
	if ((codec == "h264" || (codec == "libav" && libav_video_codec == "libx264")) && mbps > 245760.0)
	{
		LOG(1, "Overriding H.264 level 4.2");
		level = "4.2";
	}

#ifndef DISABLE_RPI_FEATURES
	if (strcasecmp(sync_.c_str(), "off") == 0)
		sync = 0;
	else if (strcasecmp(sync_.c_str(), "server") == 0)
		sync = 1;
	else if (strcasecmp(sync_.c_str(), "client") == 0)
		sync = 2;
	else
		throw std::runtime_error("incorrect sync value " + sync_);
#endif

	return true;
}

void OptsInternal::PrintVideo() const
{
	std::cerr << "    bitrate: " << bitrate.kbps() << "kbps" << std::endl;
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
#ifndef DISABLE_RPI_FEATURES
	std::cerr << "    sync: " << sync << std::endl;
#endif
}

bool OptsInternal::ParseStill()
{
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
	else if (strcasecmp(encoding.c_str(), "rgb") == 0 || strcasecmp(encoding.c_str(), "rgb24") == 0)
		encoding = "rgb24";
	else if (strcasecmp(encoding.c_str(), "rgb48") == 0)
		encoding = "rgb48";
	else if (strcasecmp(encoding.c_str(), "png") == 0)
		encoding = "png";
	else if (strcasecmp(encoding.c_str(), "bmp") == 0)
		encoding = "bmp";
	else
		throw std::runtime_error("invalid encoding format " + encoding);

	return true;
}

void OptsInternal::PrintStill() const
{
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
