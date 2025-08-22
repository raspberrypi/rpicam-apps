/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_app.cpp - base class for libcamera apps.
 */

#include "preview/preview.hpp"

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "core/options.hpp"

#include <cmath>
#include <fcntl.h>
#include <stdlib.h>

#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/dma-buf.h>
#include <linux/videodev2.h>

#include <libcamera/base/shared_fd.h>
#include <libcamera/orientation.h>

unsigned int RPiCamApp::verbosity = 1;

static libcamera::PixelFormat mode_to_pixel_format(Mode const &mode)
{
	// The saving grace here is that we can ignore the Bayer order and return anything -
	// our pipeline handler will give us back the order that works, whilst respecting the
	// bit depth and packing. We may get a "stream adjusted" message, which we can ignore.

	static std::vector<std::pair<Mode, libcamera::PixelFormat>> table = {
		{ Mode(0, 0, 8, false), libcamera::formats::SBGGR8 },
		{ Mode(0, 0, 8, true), libcamera::formats::SBGGR8 },
		{ Mode(0, 0, 10, false), libcamera::formats::SBGGR10 },
		{ Mode(0, 0, 10, true), libcamera::formats::SBGGR10_CSI2P },
		{ Mode(0, 0, 12, false), libcamera::formats::SBGGR12 },
		{ Mode(0, 0, 12, true), libcamera::formats::SBGGR12_CSI2P },
		{ Mode(0, 0, 14, false), libcamera::formats::SBGGR14 },
		{ Mode(0, 0, 14, true), libcamera::formats::SBGGR14_CSI2P },
		{ Mode(0, 0, 16, false), libcamera::formats::SBGGR16 },
		{ Mode(0, 0, 16, true), libcamera::formats::SBGGR16 },
	};

	auto it = std::find_if(table.begin(), table.end(), [&mode] (auto &m) { return mode.bit_depth == m.first.bit_depth && mode.packed == m.first.packed; });
	if (it != table.end())
		return it->second;

	return libcamera::formats::SBGGR12_CSI2P;
}

static void set_pipeline_configuration(Platform platform)
{
	// Respect any pre-existing value in the environment variable.
	char const *existing_config = getenv("LIBCAMERA_RPI_CONFIG_FILE");
	if (existing_config && existing_config[0])
		return;

	// Otherwise point it at whichever of these we find first (if any) for the given platform.
	static const std::vector<std::pair<Platform, std::string>> config_files = {
		{ Platform::VC4, "/usr/local/share/libcamera/pipeline/rpi/vc4/rpi_apps.yaml" },
		{ Platform::VC4, "/usr/share/libcamera/pipeline/rpi/vc4/rpi_apps.yaml" },
	};

	for (auto &config_file : config_files)
	{
		struct stat info;
		if (config_file.first == platform && stat(config_file.second.c_str(), &info) == 0)
		{
			setenv("LIBCAMERA_RPI_CONFIG_FILE", config_file.second.c_str(), 1);
			break;
		}
	}
}

RPiCamApp::RPiCamApp(std::unique_ptr<Options> opts)
	: options_(std::move(opts)), controls_(controls::controls), post_processor_(this)
{
	if (!options_)
		options_ = std::make_unique<Options>();

	options_->SetApp(this);

	Platform platform = options_->GetPlatform();
	if (platform == Platform::LEGACY)
	{
		// If we definitely appear to be running the old camera stack, complain and give up.
		fprintf(stderr, "ERROR: the system appears to be configured for the legacy camera stack\n");
		exit(-1);
	}
	else if (platform == Platform::UNKNOWN)
	{
		fprintf(stderr, "ERROR: rpicam-apps currently only supports the Raspberry Pi platforms.\n"
						"Contributions for other platforms are welcome at https://github.com/raspberrypi/rpicam-apps.\n");
		exit(-1);
	}

	set_pipeline_configuration(platform);
}

RPiCamApp::~RPiCamApp()
{
	if (!options_->Get().help)
		LOG(2, "Closing RPiCam application"
				   << "(frames displayed " << preview_frames_displayed_ << ", dropped " << preview_frames_dropped_
				   << ")");
	StopCamera();
	Teardown();
	CloseCamera();
}

void RPiCamApp::initCameraManager()
{
	camera_manager_.reset();
	camera_manager_ = std::make_unique<CameraManager>();
	int ret = camera_manager_->start();
	if (ret)
		throw std::runtime_error("camera manager failed to start, code " + std::to_string(-ret));
}

std::string const &RPiCamApp::CameraId() const
{
	return camera_->id();
}

std::string RPiCamApp::CameraModel() const
{
	auto model = camera_->properties().get(properties::Model);
	return model ? *model : camera_->id();
}

void RPiCamApp::OpenCamera()
{
	// Make a preview window.
	preview_ = std::unique_ptr<Preview>(make_preview(RPiCamApp::GetOptions()));
	preview_->SetDoneCallback(std::bind(&RPiCamApp::previewDoneCallback, this, std::placeholders::_1));

	LOG(2, "Opening camera...");

	if (!camera_manager_)
		initCameraManager();

	std::vector<std::shared_ptr<libcamera::Camera>> cameras = GetCameras();
	if (cameras.size() == 0)
		throw std::runtime_error("no cameras available");

	if (options_->Get().camera >= cameras.size())
		throw std::runtime_error("selected camera is not available");

	std::string const &cam_id = cameras[options_->Get().camera]->id();
	camera_ = camera_manager_->get(cam_id);
	if (!camera_)
		throw std::runtime_error("failed to find camera " + cam_id);

	if (camera_->acquire())
		throw std::runtime_error("failed to acquire camera " + cam_id);
	camera_acquired_ = true;

	LOG(2, "Acquired camera " << cam_id);

	if (!options_->Get().post_process_file.empty())
	{
		post_processor_.LoadModules(options_->Get().post_process_libs);
		post_processor_.Read(options_->Get().post_process_file);
	}
	// The queue takes over ownership from the post-processor.
	post_processor_.SetCallback(
		[this](CompletedRequestPtr &r) { this->msg_queue_.Post(Msg(MsgType::RequestComplete, std::move(r))); });

	// We're going to make a list of all the available sensor modes, but we only populate
	// the framerate field if the user has requested a framerate (as this requires us actually
	// to configure the sensor, which is otherwise best avoided).

	std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({ libcamera::StreamRole::Raw });
	const libcamera::StreamFormats &formats = config->at(0).formats();

	bool log_env_set = getenv("LIBCAMERA_LOG_LEVELS");
	// Suppress log messages when enumerating camera modes.
	if (!log_env_set)
	{
		libcamera::logSetLevel("RPI", "ERROR");
		libcamera::logSetLevel("Camera", "ERROR");
	}

	for (const auto &pix : formats.pixelformats())
	{
		for (const auto &size : formats.sizes(pix))
		{
			double framerate = 0;
			if (options_->Get().framerate)
			{
				SensorMode sensorMode(size, pix, 0);
				config->at(0).size = size;
				config->at(0).pixelFormat = pix;
				config->sensorConfig = libcamera::SensorConfiguration();
				config->sensorConfig->outputSize = size;
				config->sensorConfig->bitDepth = sensorMode.depth();
				config->validate();
				camera_->configure(config.get());
				auto fd_ctrl = camera_->controls().find(&controls::FrameDurationLimits);
				framerate = 1.0e6 / fd_ctrl->second.min().get<int64_t>();
			}
			sensor_modes_.emplace_back(size, pix, framerate);
		}
	}

	if (!log_env_set)
	{
		libcamera::logSetLevel("RPI", "INFO");
		libcamera::logSetLevel("Camera", "INFO");
	}
}

void RPiCamApp::CloseCamera()
{
	preview_.reset();

	if (camera_acquired_)
		camera_->release();
	camera_acquired_ = false;

	camera_.reset();

	camera_manager_.reset();

	if (!options_->Get().help)
		LOG(2, "Camera closed");
}

Mode RPiCamApp::selectMode(const Mode &mode) const
{
	auto scoreFormat = [](double desired, double actual) -> double
	{
		double score = desired - actual;
		// Smaller desired dimensions are preferred.
		if (score < 0.0)
			score = (-score) / 8;
		// Penalise non-exact matches.
		if (actual != desired)
			score *= 2;

		return score;
	};

	constexpr float penalty_AR = 1500.0;
	constexpr float penalty_BD = 500.0;
	constexpr float penalty_FPS = 2000.0;

	double best_score = std::numeric_limits<double>::max(), score;
	SensorMode best_mode;

	LOG(1, "Mode selection for " << mode.ToString());
	for (const auto &sensor_mode : sensor_modes_)
	{
		double reqAr = static_cast<double>(mode.width) / mode.height;
		double fmtAr = static_cast<double>(sensor_mode.size.width) / sensor_mode.size.height;

		// Similar scoring mechanism that our pipeline handler does internally.
		score = scoreFormat(mode.width, sensor_mode.size.width);
		score += scoreFormat(mode.height, sensor_mode.size.height);
		score += penalty_AR * scoreFormat(reqAr, fmtAr);
		if (mode.framerate)
			score += penalty_FPS * std::abs(mode.framerate - std::min(sensor_mode.fps, mode.framerate));
		score += penalty_BD * abs((int)(mode.bit_depth - sensor_mode.depth()));

		if (score <= best_score)
		{
			best_score = score;
			best_mode.size = sensor_mode.size;
			best_mode.format = sensor_mode.format;
		}

		LOG(1, "    " << sensor_mode.ToString() << " - Score: " << score);
	}

	return { best_mode.size.width, best_mode.size.height, best_mode.depth(), mode.packed };
}

void RPiCamApp::ConfigureViewfinder()
{
	LOG(2, "Configuring viewfinder...");

	int lores_stream_num = 0, raw_stream_num = 0;
	bool have_lores_stream = options_->Get().lores_width && options_->Get().lores_height;

	StreamRoles stream_roles = { StreamRole::Viewfinder };
	int stream_num = 1;
	if (have_lores_stream)
		stream_roles.push_back(StreamRole::Viewfinder), lores_stream_num = stream_num++;
	if (!options_->Get().no_raw)
		stream_roles.push_back(StreamRole::Raw), raw_stream_num = stream_num++;

	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate viewfinder configuration");

	Size size(1280, 960);
	auto area = camera_->properties().get(properties::PixelArrayActiveAreas);
	if (options_->Get().viewfinder_width && options_->Get().viewfinder_height)
		size = Size(options_->Get().viewfinder_width, options_->Get().viewfinder_height);
	else if (area)
	{
		// The idea here is that most sensors will have a 2x2 binned mode that
		// we can pick up. If it doesn't, well, you can always specify the size
		// you want exactly with the viewfinder_width/height options_->Get().
		size = (*area)[0].size() / 2;
		// If width and height were given, we might be switching to capture
		// afterwards - so try to match the field of view.
		if (options_->Get().width && options_->Get().height)
			size = size.boundedToAspectRatio(Size(options_->Get().width, options_->Get().height));
		size.alignDownTo(2, 2); // YUV420 will want to be even
		LOG(2, "Viewfinder size chosen is " << size.toString());
	}

	// Finally trim the image size to the largest that the preview can handle.
	Size max_size;
	preview_->MaxImageSize(max_size.width, max_size.height);
	if (max_size.width && max_size.height)
	{
		size.boundTo(max_size.boundedToAspectRatio(size)).alignDownTo(2, 2);
		LOG(2, "Final viewfinder size is " << size.toString());
	}

	// Now we get to override any of the default settings from the options_->Get().
	configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
	configuration_->at(0).size = size;
	if (options_->Get().viewfinder_buffer_count > 0)
		configuration_->at(0).bufferCount = options_->Get().viewfinder_buffer_count;

	if (have_lores_stream)
	{
		Size lores_size(options_->Get().lores_width, options_->Get().lores_height);
		lores_size.alignDownTo(2, 2);
		if (lores_size.width > size.width || lores_size.height > size.height)
			throw std::runtime_error("Low res image larger than viewfinder");
		configuration_->at(lores_stream_num).pixelFormat = lores_format_;
		configuration_->at(lores_stream_num).size = lores_size;
		configuration_->at(lores_stream_num).bufferCount = configuration_->at(0).bufferCount;
		configuration_->at(lores_stream_num).colorSpace = configuration_->at(0).colorSpace;
	}

	if (!options_->Get().no_raw)
	{
		options_->Set().viewfinder_mode.update(size, options_->Get().framerate);
		options_->Set().viewfinder_mode = selectMode(options_->Get().viewfinder_mode);

		configuration_->at(raw_stream_num).size = options_->Get().viewfinder_mode.Size();
		configuration_->at(raw_stream_num).pixelFormat = mode_to_pixel_format(options_->Get().viewfinder_mode);
		configuration_->at(raw_stream_num).bufferCount = configuration_->at(0).bufferCount;
		configuration_->sensorConfig = libcamera::SensorConfiguration();
		configuration_->sensorConfig->outputSize = options_->Get().viewfinder_mode.Size();
		configuration_->sensorConfig->bitDepth = options_->Get().viewfinder_mode.bit_depth;
	}

	configuration_->orientation = libcamera::Orientation::Rotate0 * options_->Get().transform;

	post_processor_.AdjustConfig("viewfinder", &configuration_->at(0));

	configureDenoise(options_->Get().denoise == "auto" ? "cdn_off" : options_->Get().denoise);
	setupCapture();

	streams_["viewfinder"] = configuration_->at(0).stream();
	if (have_lores_stream)
		streams_["lores"] = configuration_->at(lores_stream_num).stream();
	if (!options_->Get().no_raw)
		streams_["raw"] = configuration_->at(raw_stream_num).stream();

	post_processor_.Configure();

	LOG(2, "Viewfinder setup complete");
}

void RPiCamApp::ConfigureZsl(unsigned int still_flags)
{
	LOG(2, "Configuring ZSL...");

	StreamRoles stream_roles = { StreamRole::StillCapture, StreamRole::Viewfinder };
	if (!options_->Get().no_raw)
		stream_roles.push_back(StreamRole::Raw);

	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate viewfinder configuration");

	// Now we get to override any of the default settings from the options_->Get().
	if (still_flags & FLAG_STILL_BGR)
		configuration_->at(0).pixelFormat = libcamera::formats::BGR888;
	else if (still_flags & FLAG_STILL_RGB)
		configuration_->at(0).pixelFormat = libcamera::formats::RGB888;
	else
		configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
	if (options_->Get().buffer_count > 0)
		configuration_->at(0).bufferCount = options_->Get().buffer_count;
	else
		// Use the viewfinder stream buffer count if none has been provided
		configuration_->at(0).bufferCount = configuration_->at(1).bufferCount;
	if (options_->Get().width)
		configuration_->at(0).size.width = options_->Get().width;
	if (options_->Get().height)
		configuration_->at(0).size.height = options_->Get().height;
	configuration_->at(0).colorSpace = libcamera::ColorSpace::Sycc;
	configuration_->orientation = libcamera::Orientation::Rotate0 * options_->Get().transform;

	post_processor_.AdjustConfig("still", &configuration_->at(0));

	if (!options_->Get().no_raw)
	{
		options_->Set().mode.update(configuration_->at(0).size, options_->Get().framerate);
		options_->Set().mode = selectMode(options_->Get().mode);

		configuration_->at(2).size = options_->Get().mode.Size();
		configuration_->at(2).pixelFormat = mode_to_pixel_format(options_->Get().mode);
		configuration_->sensorConfig = libcamera::SensorConfiguration();
		configuration_->sensorConfig->outputSize = options_->Get().mode.Size();
		configuration_->sensorConfig->bitDepth = options_->Get().mode.bit_depth;
		configuration_->at(2).bufferCount = configuration_->at(0).bufferCount;
	}

	Size size(1280, 960);
	auto area = camera_->properties().get(properties::PixelArrayActiveAreas);
	if (options_->Get().viewfinder_width && options_->Get().viewfinder_height)
		size = Size(options_->Get().viewfinder_width, options_->Get().viewfinder_height);
	else if (area)
	{
		// The idea here is that most sensors will have a 2x2 binned mode that
		// we can pick up. If it doesn't, well, you can always specify the size
		// you want exactly with the viewfinder_width/height options_->Get().
		size = (*area)[0].size() / 2;
		// If width and height were given, we might be switching to capture
		// afterwards - so try to match the field of view.
		if (options_->Get().width && options_->Get().height)
			size = size.boundedToAspectRatio(Size(options_->Get().width, options_->Get().height));
		size.alignDownTo(2, 2); // YUV420 will want to be even
		LOG(2, "Viewfinder size chosen is " << size.toString());
	}

	// Finally trim the image size to the largest that the preview can handle.
	Size max_size;
	preview_->MaxImageSize(max_size.width, max_size.height);
	if (max_size.width && max_size.height)
	{
		size.boundTo(max_size.boundedToAspectRatio(size)).alignDownTo(2, 2);
		LOG(2, "Final viewfinder size is " << size.toString());
	}

	// Now we get to override any of the default settings from the options_->Get().
	configuration_->at(1).pixelFormat = libcamera::formats::YUV420;
	configuration_->at(1).size = size;
	configuration_->at(1).bufferCount = configuration_->at(0).bufferCount;

	configuration_->orientation = libcamera::Orientation::Rotate0 * options_->Get().transform;

	post_processor_.AdjustConfig("viewfinder", &configuration_->at(1));

	configureDenoise(options_->Get().denoise == "auto" ? "cdn_hq" : options_->Get().denoise);
	setupCapture();

	streams_["still"] = configuration_->at(0).stream();
	streams_["viewfinder"] = configuration_->at(1).stream();
	if (!options_->Get().no_raw)
		streams_["raw"] = configuration_->at(2).stream();

	post_processor_.Configure();

	LOG(2, "ZSL setup complete");
}

void RPiCamApp::ConfigureStill(unsigned int flags)
{
	LOG(2, "Configuring still capture...");

	// Always request a raw stream as this forces the full resolution capture mode,
	// unless the no-raw option is used.
	// (options_->Get().mode can override the choice of camera mode, however.)
	StreamRoles stream_roles = { StreamRole::StillCapture };
	if (!options_->Get().no_raw)
		stream_roles.push_back(StreamRole::Raw);
	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate still capture configuration");

	// Now we get to override any of the default settings from the options_->Get().
	if (flags & FLAG_STILL_BGR)
		configuration_->at(0).pixelFormat = libcamera::formats::BGR888;
	else if (flags & FLAG_STILL_RGB)
		configuration_->at(0).pixelFormat = libcamera::formats::RGB888;
	else if (flags & FLAG_STILL_BGR48)
		configuration_->at(0).pixelFormat = libcamera::formats::BGR161616;
	else if (flags & FLAG_STILL_RGB48)
		configuration_->at(0).pixelFormat = libcamera::formats::RGB161616;
	else
		configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
	if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_DOUBLE_BUFFER)
		configuration_->at(0).bufferCount = 2;
	else if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_TRIPLE_BUFFER)
		configuration_->at(0).bufferCount = 3;
	else if (options_->Get().buffer_count > 0)
		configuration_->at(0).bufferCount = options_->Get().buffer_count;
	if (options_->Get().width)
		configuration_->at(0).size.width = options_->Get().width;
	if (options_->Get().height)
		configuration_->at(0).size.height = options_->Get().height;
	configuration_->at(0).colorSpace = libcamera::ColorSpace::Sycc;
	configuration_->orientation = libcamera::Orientation::Rotate0 * options_->Get().transform;

	post_processor_.AdjustConfig("still", &configuration_->at(0));

	if (!options_->Get().no_raw)
	{
		options_->Set().mode.update(configuration_->at(0).size, options_->Get().framerate);
		options_->Set().mode = selectMode(options_->Get().mode);

		configuration_->at(1).size = options_->Get().mode.Size();
		configuration_->at(1).pixelFormat = mode_to_pixel_format(options_->Get().mode);
		configuration_->sensorConfig = libcamera::SensorConfiguration();
		configuration_->sensorConfig->outputSize = options_->Get().mode.Size();
		configuration_->sensorConfig->bitDepth = options_->Get().mode.bit_depth;
		configuration_->at(1).bufferCount = configuration_->at(0).bufferCount;
	}

	configureDenoise(options_->Get().denoise == "auto" ? "cdn_hq" : options_->Get().denoise);
	setupCapture();

	streams_["still"] = configuration_->at(0).stream();
	if (!options_->Get().no_raw)
		streams_["raw"] = configuration_->at(1).stream();

	post_processor_.Configure();

	LOG(2, "Still capture setup complete");
}

void RPiCamApp::ConfigureVideo(unsigned int flags)
{
	LOG(2, "Configuring video...");

	bool have_lores_stream = options_->Get().lores_width && options_->Get().lores_height;
	StreamRoles stream_roles = { StreamRole::VideoRecording };
	int lores_index = 1;
	if (!options_->Get().no_raw)
		stream_roles.push_back(StreamRole::Raw), lores_index++;
	if (have_lores_stream)
		stream_roles.push_back(StreamRole::Viewfinder);
	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate video configuration");

	// Now we get to override any of the default settings from the options_->Get().
	StreamConfiguration &cfg = configuration_->at(0);
	cfg.pixelFormat = libcamera::formats::YUV420;
	cfg.bufferCount = 6; // 6 buffers is better than 4
	if (options_->Get().buffer_count > 0)
		cfg.bufferCount = options_->Get().buffer_count;
	if (options_->Get().width)
		cfg.size.width = options_->Get().width;
	if (options_->Get().height)
		cfg.size.height = options_->Get().height;
	if (flags & FLAG_VIDEO_JPEG_COLOURSPACE)
		cfg.colorSpace = libcamera::ColorSpace::Sycc;
	else if (cfg.size.width >= 1280 || cfg.size.height >= 720)
		cfg.colorSpace = libcamera::ColorSpace::Rec709;
	else
		cfg.colorSpace = libcamera::ColorSpace::Smpte170m;
	configuration_->orientation = libcamera::Orientation::Rotate0 * options_->Get().transform;

	post_processor_.AdjustConfig("video", &configuration_->at(0));

	if (!options_->Get().no_raw)
	{
		options_->Set().mode.update(configuration_->at(0).size, options_->Get().framerate);
		options_->Set().mode = selectMode(options_->Get().mode);

		configuration_->at(1).size = options_->Get().mode.Size();
		configuration_->at(1).pixelFormat = mode_to_pixel_format(options_->Get().mode);
		configuration_->sensorConfig = libcamera::SensorConfiguration();
		configuration_->sensorConfig->outputSize = options_->Get().mode.Size();
		configuration_->sensorConfig->bitDepth = options_->Get().mode.bit_depth;
		configuration_->at(1).bufferCount = configuration_->at(0).bufferCount;
	}

	if (have_lores_stream)
	{
		Size lores_size(options_->Get().lores_width, options_->Get().lores_height);
		lores_size.alignDownTo(2, 2);
		if (lores_size.width > configuration_->at(0).size.width ||
			lores_size.height > configuration_->at(0).size.height)
			throw std::runtime_error("Low res image larger than video");
		configuration_->at(lores_index).pixelFormat = lores_format_;
		configuration_->at(lores_index).size = lores_size;
		configuration_->at(lores_index).bufferCount = configuration_->at(0).bufferCount;
		configuration_->at(lores_index).colorSpace = configuration_->at(0).colorSpace;
	}
	configuration_->orientation = libcamera::Orientation::Rotate0 * options_->Get().transform;

	configureDenoise(options_->Get().denoise == "auto" ? "cdn_fast" : options_->Get().denoise);
	setupCapture();

	streams_["video"] = configuration_->at(0).stream();
	if (!options_->Get().no_raw)
		streams_["raw"] = configuration_->at(1).stream();
	if (have_lores_stream)
		streams_["lores"] = configuration_->at(lores_index).stream();

	post_processor_.Configure();

	LOG(2, "Video setup complete");
}

void RPiCamApp::Teardown()
{
	stopPreview();

	post_processor_.Teardown();

	if (!options_->Get().help)
		LOG(2, "Tearing down requests, buffers and configuration");

	for (auto &iter : mapped_buffers_)
	{
		// assert(iter.first->planes().size() == iter.second.size());
		// for (unsigned i = 0; i < iter.first->planes().size(); i++)
		for (auto &span : iter.second)
			munmap(span.data(), span.size());
	}
	mapped_buffers_.clear();

	configuration_.reset();

	frame_buffers_.clear();

	streams_.clear();
}

void RPiCamApp::StartCamera()
{
	// This makes all the Request objects that we shall need.
	makeRequests();

	// Build a list of initial controls that we must set in the camera before starting it.
	// We don't overwrite anything the application may have set before calling us.
	if (!controls_.get(controls::ScalerCrop) && !controls_.get(controls::rpi::ScalerCrops))
	{
		const Rectangle sensor_area = camera_->controls().at(&controls::ScalerCrop).max().get<Rectangle>();
		const Rectangle default_crop = camera_->controls().at(&controls::ScalerCrop).def().get<Rectangle>();
		std::vector<Rectangle> crops;

		if (options_->Get().roi_width != 0 && options_->Get().roi_height != 0)
		{
			int x = options_->Get().roi_x * sensor_area.width;
			int y = options_->Get().roi_y * sensor_area.height;
			unsigned int w = options_->Get().roi_width * sensor_area.width;
			unsigned int h = options_->Get().roi_height * sensor_area.height;
			crops.push_back({ x, y, w, h });
			crops.back().translateBy(sensor_area.topLeft());
		}
		else
		{
			crops.push_back(default_crop);
		}

		LOG(2, "Using crop (main) " << crops.back().toString());

		if (options_->Get().lores_width != 0 && options_->Get().lores_height != 0 && !options_->Get().lores_par)
		{
			crops.push_back(crops.back());
			LOG(2, "Using crop (lores) " << crops.back().toString());
		}

		if (options_->GetPlatform() == Platform::VC4)
			controls_.set(controls::ScalerCrop, crops[0]);
		else
			controls_.set(controls::rpi::ScalerCrops, libcamera::Span<const Rectangle>(crops.data(), crops.size()));
	}

	if (!controls_.get(controls::AfWindows) && !controls_.get(controls::AfMetering) &&
		options_->Get().afWindow_width != 0 && options_->Get().afWindow_height != 0)
	{
		Rectangle sensor_area = camera_->controls().at(&controls::ScalerCrop).max().get<Rectangle>();
		int x = options_->Get().afWindow_x * sensor_area.width;
		int y = options_->Get().afWindow_y * sensor_area.height;
		int w = options_->Get().afWindow_width * sensor_area.width;
		int h = options_->Get().afWindow_height * sensor_area.height;
		Rectangle afwindows_rectangle[1];
		afwindows_rectangle[0] = Rectangle(x, y, w, h);
		afwindows_rectangle[0].translateBy(sensor_area.topLeft());
		LOG(2, "Using AfWindow " << afwindows_rectangle[0].toString());
		//activate the AfMeteringWindows
		controls_.set(controls::AfMetering, controls::AfMeteringWindows);
		//set window
		controls_.set(controls::AfWindows, afwindows_rectangle);
	}

	// Framerate is a bit weird. If it was set programmatically, we go with that, but
	// otherwise it applies only to preview/video modes. For stills capture we set it
	// as long as possible so that we get whatever the exposure profile wants.
	if (!controls_.get(controls::FrameDurationLimits))
	{
		if (StillStream())
			controls_.set(controls::FrameDurationLimits,
						  libcamera::Span<const int64_t, 2>({ INT64_C(100), INT64_C(1000000000) }));
		else if (!options_->Get().framerate || options_->Get().framerate.value() > 0)
		{
			int64_t frame_time = 1000000 / options_->Get().framerate.value_or(DEFAULT_FRAMERATE); // in us
			controls_.set(controls::FrameDurationLimits,
						  libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
		}
	}

	if (!controls_.get(controls::ExposureTime) && options_->Get().shutter)
	{
		controls_.set(controls::ExposureTimeMode, controls::ExposureTimeModeManual);
		controls_.set(controls::ExposureTime, options_->Get().shutter.get<std::chrono::microseconds>());
	}
	if (!controls_.get(controls::AnalogueGain) && options_->Get().gain)
	{
		controls_.set(controls::AnalogueGainMode, controls::AnalogueGainModeManual);
		controls_.set(controls::AnalogueGain, options_->Get().gain);
	}
	if (!controls_.get(controls::AeMeteringMode))
		controls_.set(controls::AeMeteringMode, options_->Get().metering_index);
	if (!controls_.get(controls::AeExposureMode))
		controls_.set(controls::AeExposureMode, options_->Get().exposure_index);
	if (!controls_.get(controls::ExposureValue))
		controls_.set(controls::ExposureValue, options_->Get().ev);
	if (!controls_.get(controls::AwbMode))
		controls_.set(controls::AwbMode, options_->Get().awb_index);
	if (!controls_.get(controls::ColourGains) && options_->Get().awb_gain_r && options_->Get().awb_gain_b)
		controls_.set(controls::ColourGains,
					  libcamera::Span<const float, 2>({ options_->Get().awb_gain_r, options_->Get().awb_gain_b }));
	if (!controls_.get(controls::ColourCorrectionMatrix) && !options_->Get().ccm.empty()) {
		if (!controls_.get(controls::ColourGains))
			LOG_ERROR("WARNING: cannot set colour correction matrix without explicit AWB gains (--awbgains)");
		else
		{
			libcamera::Span<const float, 9> span(options_->Get().ccm_values);
			controls_.set(controls::ColourCorrectionMatrix, span);
		}
	}
	if (!controls_.get(controls::Brightness))
		controls_.set(controls::Brightness, options_->Get().brightness);
	if (!controls_.get(controls::Contrast))
		controls_.set(controls::Contrast, options_->Get().contrast);
	if (!controls_.get(controls::Saturation))
		controls_.set(controls::Saturation, options_->Get().saturation);
	if (!controls_.get(controls::Sharpness))
		controls_.set(controls::Sharpness, options_->Get().sharpness);
	if (!controls_.get(controls::HdrMode) && (options_->Get().hdr == "auto" || options_->Get().hdr == "single-exp"))
		controls_.set(controls::HdrMode, controls::HdrModeSingleExposure);

	// AF Controls, where supported and not already set
	if (!controls_.get(controls::AfMode) && camera_->controls().count(&controls::AfMode) > 0)
	{
		int afm = options_->Get().afMode_index;
		if (afm == -1)
		{
			// Choose a default AF mode based on other options
			if (options_->Get().lens_position || options_->Get().set_default_lens_position ||
				options_->Get().af_on_capture)
				afm = controls::AfModeManual;
			else
				afm = camera_->controls().at(&controls::AfMode).max().get<int>();
		}
		controls_.set(controls::AfMode, afm);
	}
	if (!controls_.get(controls::AfRange) && camera_->controls().count(&controls::AfRange) > 0)
		controls_.set(controls::AfRange, options_->Get().afRange_index);
	if (!controls_.get(controls::AfSpeed) && camera_->controls().count(&controls::AfSpeed) > 0)
		controls_.set(controls::AfSpeed, options_->Get().afSpeed_index);

	if (controls_.get(controls::AfMode).value_or(controls::AfModeManual) == controls::AfModeAuto)
	{
		// When starting a viewfinder or video stream in AF "auto" mode,
		// trigger a scan now (but don't move the lens when capturing a still).
		// If an application requires more control over AF triggering, it may
		// override this behaviour with prior settings of AfMode or AfTrigger.
		if (!StillStream() && !controls_.get(controls::AfTrigger))
			controls_.set(controls::AfTrigger, controls::AfTriggerStart);
	}
	else if ((options_->Get().lens_position || options_->Get().set_default_lens_position) &&
			 camera_->controls().count(&controls::LensPosition) > 0 && !controls_.get(controls::LensPosition))
	{
		float f;
		if (options_->Get().lens_position)
			f = options_->Get().lens_position.value();
		else
			f = camera_->controls().at(&controls::LensPosition).def().get<float>();
		LOG(2, "Setting LensPosition: " << f);
		controls_.set(controls::LensPosition, f);
	}

	if (options_->Get().flicker_period && !controls_.get(controls::AeFlickerMode) &&
		camera_->controls().find(&controls::AeFlickerMode) != camera_->controls().end() &&
		camera_->controls().find(&controls::AeFlickerPeriod) != camera_->controls().end())
	{
		controls_.set(controls::AeFlickerMode, controls::FlickerManual);
		controls_.set(controls::AeFlickerPeriod, options_->Get().flicker_period.get<std::chrono::microseconds>());
	}

	if (camera_->start(&controls_))
		throw std::runtime_error("failed to start camera");
	controls_.clear();
	camera_started_ = true;
	last_timestamp_ = 0;

	post_processor_.Start();

	camera_->requestCompleted.connect(this, &RPiCamApp::requestComplete);

	for (std::unique_ptr<Request> &request : requests_)
	{
		if (camera_->queueRequest(request.get()) < 0)
			throw std::runtime_error("Failed to queue request");
	}

	LOG(2, "Camera started!");
}

void RPiCamApp::StopCamera()
{
	{
		// We don't want QueueRequest to run asynchronously while we stop the camera.
		std::lock_guard<std::mutex> lock(camera_stop_mutex_);
		if (camera_started_)
		{
			if (camera_->stop())
				throw std::runtime_error("failed to stop camera");

			post_processor_.Stop();

			camera_started_ = false;
		}
	}

	if (camera_)
		camera_->requestCompleted.disconnect(this, &RPiCamApp::requestComplete);

	// An application might be holding a CompletedRequest, so queueRequest will get
	// called to delete it later, but we need to know not to try and re-queue it.
	completed_requests_.clear();

	msg_queue_.Clear();

	requests_.clear();

	controls_.clear(); // no need for mutex here

	if (!options_->Get().help)
		LOG(2, "Camera stopped!");
}

RPiCamApp::Msg RPiCamApp::Wait()
{
	return msg_queue_.Wait();
}

void RPiCamApp::queueRequest(CompletedRequest *completed_request)
{
	BufferMap buffers(std::move(completed_request->buffers));

	// This function may run asynchronously so needs protection from the
	// camera stopping at the same time.
	std::lock_guard<std::mutex> stop_lock(camera_stop_mutex_);

	// An application could be holding a CompletedRequest while it stops and re-starts
	// the camera, after which we don't want to queue another request now.
	bool request_found;
	{
		std::lock_guard<std::mutex> lock(completed_requests_mutex_);
		auto it = completed_requests_.find(completed_request);
		if (it != completed_requests_.end())
		{
			request_found = true;
			completed_requests_.erase(it);
		}
		else
			request_found = false;
	}

	Request *request = completed_request->request;
	delete completed_request;
	assert(request);

	if (!camera_started_ || !request_found)
		return;

	for (auto const &p : buffers)
	{
		struct dma_buf_sync dma_sync {};
		dma_sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;

		auto it = mapped_buffers_.find(p.second);
		if (it == mapped_buffers_.end())
			throw std::runtime_error("failed to identify queue request buffer");

		int ret = ::ioctl(p.second->planes()[0].fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
		if (ret)
			throw std::runtime_error("failed to sync dma buf on queue request");

		if (request->addBuffer(p.first, p.second) < 0)
			throw std::runtime_error("failed to add buffer to request in QueueRequest");
	}

	{
		std::lock_guard<std::mutex> lock(control_mutex_);
		request->controls() = std::move(controls_);
	}

	if (camera_->queueRequest(request) < 0)
		throw std::runtime_error("failed to queue request");
}

void RPiCamApp::PostMessage(MsgType &t, MsgPayload &p)
{
	msg_queue_.Post(Msg(t, std::move(p)));
}

libcamera::Stream *RPiCamApp::GetStream(std::string const &name, StreamInfo *info) const
{
	auto it = streams_.find(name);
	if (it == streams_.end())
		return nullptr;
	if (info)
		*info = GetStreamInfo(it->second);
	return it->second;
}

libcamera::Stream *RPiCamApp::ViewfinderStream(StreamInfo *info) const
{
	return GetStream("viewfinder", info);
}

libcamera::Stream *RPiCamApp::StillStream(StreamInfo *info) const
{
	return GetStream("still", info);
}

libcamera::Stream *RPiCamApp::RawStream(StreamInfo *info) const
{
	return GetStream("raw", info);
}

libcamera::Stream *RPiCamApp::VideoStream(StreamInfo *info) const
{
	return GetStream("video", info);
}

libcamera::Stream *RPiCamApp::LoresStream(StreamInfo *info) const
{
	return GetStream("lores", info);
}

libcamera::Stream *RPiCamApp::GetMainStream() const
{
	for (auto &p : streams_)
	{
		if (p.first == "viewfinder" || p.first == "still" || p.first == "video")
			return p.second;
	}

	return nullptr;
}

const libcamera::CameraManager *RPiCamApp::GetCameraManager()
{
	if (!camera_manager_)
		initCameraManager();

	return camera_manager_.get();
}

void RPiCamApp::ShowPreview(CompletedRequestPtr &completed_request, Stream *stream)
{
	std::lock_guard<std::mutex> lock(preview_item_mutex_);
	if (!preview_item_.stream)
		preview_item_ = PreviewItem(completed_request, stream); // copy the shared_ptr here
	else
		preview_frames_dropped_++;
	preview_cond_var_.notify_one();
}

void RPiCamApp::SetControls(const ControlList &controls)
{
	std::lock_guard<std::mutex> lock(control_mutex_);

	// Add new controls to the stored list. If a control is duplicated,
	// the value in the argument replaces the previously stored value.
	// These controls will be applied to the next StartCamera or request.
	for (const auto &c : controls)
		controls_.set(c.first, c.second);
}

StreamInfo RPiCamApp::GetStreamInfo(Stream const *stream) const
{
	StreamConfiguration const &cfg = stream->configuration();
	StreamInfo info;
	info.width = cfg.size.width;
	info.height = cfg.size.height;
	info.stride = cfg.stride;
	info.pixel_format = cfg.pixelFormat;
	info.colour_space = cfg.colorSpace;
	return info;
}

void RPiCamApp::setupCapture()
{
	// First finish setting up the configuration.

	for (auto &config : *configuration_)
		config.stride = 0;
	CameraConfiguration::Status validation = configuration_->validate();
	if (validation == CameraConfiguration::Invalid)
		throw std::runtime_error("failed to valid stream configurations");
	else if (validation == CameraConfiguration::Adjusted)
		LOG(1, "Stream configuration adjusted");

	if (camera_->configure(configuration_.get()) < 0)
		throw std::runtime_error("failed to configure streams");
	LOG(2, "Camera streams configured");

	LOG(2, "Available controls:");
	for (auto const &[id, info] : camera_->controls())
		LOG(2, "    " << id->name() << " : " << info.toString());

	// Next allocate all the buffers we need, mmap them and store them on a free list.

	for (StreamConfiguration &config : *configuration_)
	{
		Stream *stream = config.stream();
		std::vector<std::unique_ptr<FrameBuffer>> fb;

		for (unsigned int i = 0; i < config.bufferCount; i++)
		{
			std::string name("rpicam-apps" + std::to_string(i));
			libcamera::UniqueFD fd = dma_heap_.alloc(name.c_str(), config.frameSize);

			if (!fd.isValid())
				throw std::runtime_error("failed to allocate capture buffers for stream");

			std::vector<FrameBuffer::Plane> plane(1);
			plane[0].fd = libcamera::SharedFD(std::move(fd));
			plane[0].offset = 0;
			plane[0].length = config.frameSize;

			fb.push_back(std::make_unique<FrameBuffer>(plane));
			void *memory = mmap(NULL, config.frameSize, PROT_READ | PROT_WRITE, MAP_SHARED, plane[0].fd.get(), 0);
			mapped_buffers_[fb.back().get()].push_back(
						libcamera::Span<uint8_t>(static_cast<uint8_t *>(memory), config.frameSize));
		}

		frame_buffers_[stream] = std::move(fb);
	}
	LOG(2, "Buffers allocated and mapped");

	startPreview();

	// The requests will be made when StartCamera() is called.
}

void RPiCamApp::makeRequests()
{
	std::map<Stream *, std::queue<FrameBuffer *>> free_buffers;

	for (auto &kv : frame_buffers_)
	{
		free_buffers[kv.first] = {};
		for (auto &b : kv.second)
			free_buffers[kv.first].push(b.get());
	}

	while (true)
	{
		for (StreamConfiguration &config : *configuration_)
		{
			Stream *stream = config.stream();
			if (stream == configuration_->at(0).stream())
			{
				if (free_buffers[stream].empty())
				{
					LOG(2, "Requests created");
					return;
				}
				std::unique_ptr<Request> request = camera_->createRequest();
				if (!request)
					throw std::runtime_error("failed to make request");
				requests_.push_back(std::move(request));
			}
			else if (free_buffers[stream].empty())
				throw std::runtime_error("concurrent streams need matching numbers of buffers");

			FrameBuffer *buffer = free_buffers[stream].front();
			free_buffers[stream].pop();
			if (requests_.back()->addBuffer(stream, buffer) < 0)
				throw std::runtime_error("failed to add buffer to request");
		}
	}
}

void RPiCamApp::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
	{
		// If the request is cancelled while the camera is still running, it indicates
		// a hardware timeout. Let the application handle this error.
		if (camera_started_)
			msg_queue_.Post(Msg(MsgType::Timeout));

		return;
	}

	struct dma_buf_sync dma_sync {};
	dma_sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
	for (auto const &buffer_map : request->buffers())
	{
		auto it = mapped_buffers_.find(buffer_map.second);
		if (it == mapped_buffers_.end())
			throw std::runtime_error("failed to identify request complete buffer");

		int ret = ::ioctl(buffer_map.second->planes()[0].fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
		if (ret)
			throw std::runtime_error("failed to sync dma buf on request complete");
	}

	CompletedRequest *r = new CompletedRequest(sequence_++, request);
	CompletedRequestPtr payload(r, [this](CompletedRequest *cr) { this->queueRequest(cr); });
	{
		std::lock_guard<std::mutex> lock(completed_requests_mutex_);
		completed_requests_.insert(r);
	}

	// Framebuffer reports possibly being in a startup or error state, ignore these.
	if (r->buffers.begin()->second->metadata().status != libcamera::FrameMetadata::FrameSuccess)
		return;

	// We calculate the instantaneous framerate in case anyone wants it.
	// Use the sensor timestamp if possible as it ought to be less glitchy than
	// the buffer timestamps.
	auto ts = payload->metadata.get(controls::SensorTimestamp);
	uint64_t timestamp = ts ? *ts : payload->buffers.begin()->second->metadata().timestamp;
	if (last_timestamp_ == 0 || last_timestamp_ == timestamp)
		payload->framerate = 0;
	else
		payload->framerate = 1e9 / (timestamp - last_timestamp_);
	last_timestamp_ = timestamp;

	post_processor_.Process(payload); // post-processor can re-use our shared_ptr
}

void RPiCamApp::previewDoneCallback(int fd)
{
	std::lock_guard<std::mutex> lock(preview_mutex_);
	auto it = preview_completed_requests_.find(fd);
	if (it == preview_completed_requests_.end())
		throw std::runtime_error("previewDoneCallback: missing fd " + std::to_string(fd));
	preview_completed_requests_.erase(it); // drop shared_ptr reference
}

void RPiCamApp::startPreview()
{
	preview_abort_ = false;
	preview_thread_ = std::thread(&RPiCamApp::previewThread, this);
}

void RPiCamApp::stopPreview()
{
	if (!preview_thread_.joinable()) // in case never started
		return;

	{
		std::lock_guard<std::mutex> lock(preview_item_mutex_);
		preview_abort_ = true;
		preview_cond_var_.notify_one();
	}
	preview_thread_.join();
	preview_item_ = PreviewItem();
	preview_completed_requests_.clear();
}

void RPiCamApp::previewThread()
{
	while (true)
	{
		PreviewItem item;
		while (!item.stream)
		{
			std::unique_lock<std::mutex> lock(preview_item_mutex_);
			if (preview_abort_)
			{
				preview_->Reset();
				return;
			}
			else if (preview_item_.stream)
				item = std::move(preview_item_); // re-use existing shared_ptr reference
			else
				preview_cond_var_.wait(lock);
		}

		if (item.stream->configuration().pixelFormat != libcamera::formats::YUV420)
			throw std::runtime_error("Preview windows only support YUV420");

		StreamInfo info = GetStreamInfo(item.stream);
		FrameBuffer *buffer = item.completed_request->buffers[item.stream];
		BufferReadSync r(this, buffer);
		libcamera::Span span = r.Get()[0];

		// Fill the frame info with the ControlList items and ancillary bits.
		FrameInfo frame_info(item.completed_request);

		int fd = buffer->planes()[0].fd.get();
		{
			std::lock_guard<std::mutex> lock(preview_mutex_);
			// the reference to the shared_ptr moves to the map here
			preview_completed_requests_[fd] = std::move(item.completed_request);
		}
		if (preview_->Quit())
		{
			LOG(2, "Preview window has quit");
			msg_queue_.Post(Msg(MsgType::Quit));
		}
		preview_frames_displayed_++;
		preview_->Show(fd, span, info);
		if (!options_->Get().info_text.empty())
		{
			std::string s = frame_info.ToString(options_->Get().info_text);
			preview_->SetInfoText(s);
		}
	}
}

void RPiCamApp::configureDenoise(const std::string &denoise_mode)
{
	using namespace libcamera::controls::draft;

	static const std::map<std::string, NoiseReductionModeEnum> denoise_table = {
		{ "off", NoiseReductionModeOff },
		{ "cdn_off", NoiseReductionModeMinimal },
		{ "cdn_fast", NoiseReductionModeFast },
		{ "cdn_hq", NoiseReductionModeHighQuality }
	};
	NoiseReductionModeEnum denoise;

	auto const mode = denoise_table.find(denoise_mode);
	if (mode == denoise_table.end())
		throw std::runtime_error("Invalid denoise mode " + denoise_mode);
	denoise = mode->second;

	controls_.set(NoiseReductionMode, denoise);
}
