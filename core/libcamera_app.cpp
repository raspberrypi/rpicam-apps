/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_app.cpp - base class for libcamera apps.
 */

#include "preview/preview.hpp"

#include "core/frame_info.hpp"
#include "core/libcamera_app.hpp"
#include "core/options.hpp"

#include <cmath>
#include <fcntl.h>

#include <sys/ioctl.h>

#include <linux/videodev2.h>

unsigned int LibcameraApp::verbosity = 2;

// If we definitely appear to be running the old camera stack, complain and give up.
// Everything else, Pi or not, we let through.

static void check_camera_stack()
{
	int fd = open("/dev/video0", O_RDWR, 0);
	if (fd < 0)
		return;

	v4l2_capability caps;
	unsigned long request = VIDIOC_QUERYCAP;

	int ret = ioctl(fd, request, &caps);
	close(fd);

	if (ret < 0 || strcmp((char *)caps.driver, "bm2835 mmal"))
		return;

	fprintf(stderr, "ERROR: the system appears to be configured for the legacy camera stack\n");
	exit(-1);
}

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
	};

	auto it = std::find_if(table.begin(), table.end(), [&mode] (auto &m) { return mode.bit_depth == m.first.bit_depth && mode.packed == m.first.packed; });
	if (it != table.end())
		return it->second;

	return libcamera::formats::SBGGR12_CSI2P;
}

LibcameraApp::LibcameraApp(std::unique_ptr<Options> opts)
	: options_(std::move(opts)), controls_(controls::controls), post_processor_(this)
{
	check_camera_stack();

	if (!options_)
		options_ = std::make_unique<Options>();
}

LibcameraApp::~LibcameraApp()
{
	if (!options_->help)
		LOG(2, "Closing Libcamera application"
				   << "(frames displayed " << preview_frames_displayed_ << ", dropped " << preview_frames_dropped_
				   << ")");
	StopCamera();
	Teardown();
	CloseCamera();
}

std::string const &LibcameraApp::CameraId() const
{
	return camera_->id();
}

void LibcameraApp::OpenCamera()
{
	// Make a preview window.
	preview_ = std::unique_ptr<Preview>(make_preview(options_.get()));
	preview_->SetDoneCallback(std::bind(&LibcameraApp::previewDoneCallback, this, std::placeholders::_1));

	LOG(2, "Opening camera...");

	camera_manager_ = std::make_unique<CameraManager>();
	int ret = camera_manager_->start();
	if (ret)
		throw std::runtime_error("camera manager failed to start, code " + std::to_string(-ret));

	std::vector<std::shared_ptr<libcamera::Camera>> cameras = camera_manager_->cameras();
	// Do not show USB webcams as these are not supported in libcamera-apps!
	auto rem = std::remove_if(cameras.begin(), cameras.end(),
							  [](auto &cam) { return cam->id().find("/usb") != std::string::npos; });
	cameras.erase(rem, cameras.end());

	if (cameras.size() == 0)
		throw std::runtime_error("no cameras available");
	if (options_->camera >= cameras.size())
		throw std::runtime_error("selected camera is not available");

	std::string const &cam_id = cameras[options_->camera]->id();
	camera_ = camera_manager_->get(cam_id);
	if (!camera_)
		throw std::runtime_error("failed to find camera " + cam_id);

	if (camera_->acquire())
		throw std::runtime_error("failed to acquire camera " + cam_id);
	camera_acquired_ = true;

	LOG(2, "Acquired camera " << cam_id);

	if (!options_->post_process_file.empty())
		post_processor_.Read(options_->post_process_file);
	// The queue takes over ownership from the post-processor.
	post_processor_.SetCallback(
		[this](CompletedRequestPtr &r) { this->msg_queue_.Post(Msg(MsgType::RequestComplete, std::move(r))); });

	if (options_->framerate)
	{
		std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({ libcamera::StreamRole::Raw });
		const libcamera::StreamFormats &formats = config->at(0).formats();

		// Suppress log messages when enumerating camera modes.
		libcamera::logSetLevel("RPI", "ERROR");
		libcamera::logSetLevel("Camera", "ERROR");

		for (const auto &pix : formats.pixelformats())
		{
			for (const auto &size : formats.sizes(pix))
			{
				config->at(0).size = size;
				config->at(0).pixelFormat = pix;
				config->validate();
				camera_->configure(config.get());
				auto fd_ctrl = camera_->controls().find(&controls::FrameDurationLimits);
				sensor_modes_.emplace_back(size, pix, 1.0e6 / fd_ctrl->second.min().get<int64_t>());
			}
		}

		libcamera::logSetLevel("RPI", "INFO");
		libcamera::logSetLevel("Camera", "INFO");
	}
}

void LibcameraApp::CloseCamera()
{
	preview_.reset();

	if (camera_acquired_)
		camera_->release();
	camera_acquired_ = false;

	camera_.reset();

	camera_manager_.reset();

	if (!options_->help)
		LOG(2, "Camera closed");
}

Mode LibcameraApp::selectModeForFramerate(const libcamera::Size &req, double fps)
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

	LOG(1, "Mode selection:");
	for (const auto &mode : sensor_modes_)
	{
		double reqAr = static_cast<double>(req.width) / req.height;
		double fmtAr = static_cast<double>(mode.size.width) / mode.size.height;

		// Similar scoring mechanism that our pipeline handler does internally.
		score = scoreFormat(req.width, mode.size.width);
		score += scoreFormat(req.height, mode.size.height);
		score += penalty_AR * scoreFormat(reqAr, fmtAr);
		score += penalty_FPS * std::abs(fps - std::min(mode.fps, fps));
		score += penalty_BD * (16 - mode.depth());

		if (score <= best_score)
		{
			best_score = score;
			best_mode.size = mode.size;
			best_mode.format = mode.format;
		}

		LOG(1, "    " << mode.format.toString() << " " << mode.size.toString() << " - Score: " << score);
	}

	return { best_mode.size.width, best_mode.size.height, best_mode.depth(), true };
}

void LibcameraApp::ConfigureViewfinder()
{
	LOG(2, "Configuring viewfinder...");

	bool select_mode = options_->framerate && options_->framerate.value() && options_->viewfinder_mode_string.empty();
	int lores_stream_num = 0, raw_stream_num = 0;
	bool have_lores_stream = options_->lores_width && options_->lores_height;
	bool have_raw_stream = options_->viewfinder_mode.bit_depth || select_mode;

	StreamRoles stream_roles = { StreamRole::Viewfinder };
	int stream_num = 1;
	if (have_lores_stream)
		stream_roles.push_back(StreamRole::Viewfinder), lores_stream_num = stream_num++;
	if (have_raw_stream)
		stream_roles.push_back(StreamRole::Raw), raw_stream_num = stream_num++;

	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate viewfinder configuration");

	Size size(1280, 960);
	auto area = camera_->properties().get(properties::PixelArrayActiveAreas);
	if (options_->viewfinder_width && options_->viewfinder_height)
		size = Size(options_->viewfinder_width, options_->viewfinder_height);
	else if (area)
	{
		// The idea here is that most sensors will have a 2x2 binned mode that
		// we can pick up. If it doesn't, well, you can always specify the size
		// you want exactly with the viewfinder_width/height options_->
		size = (*area)[0].size() / 2;
		// If width and height were given, we might be switching to capture
		// afterwards - so try to match the field of view.
		if (options_->width && options_->height)
			size = size.boundedToAspectRatio(Size(options_->width, options_->height));
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

	// Now we get to override any of the default settings from the options_->
	configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
	configuration_->at(0).size = size;
	if (options_->viewfinder_buffer_count > 0)
		configuration_->at(0).bufferCount = options_->viewfinder_buffer_count;

	if (have_lores_stream)
	{
		Size lores_size(options_->lores_width, options_->lores_height);
		lores_size.alignDownTo(2, 2);
		if (lores_size.width > size.width || lores_size.height > size.height)
			throw std::runtime_error("Low res image larger than viewfinder");
		configuration_->at(lores_stream_num).pixelFormat = libcamera::formats::YUV420;
		configuration_->at(lores_stream_num).size = lores_size;
		configuration_->at(lores_stream_num).bufferCount = configuration_->at(0).bufferCount;
	}

	if (select_mode)
		options_->viewfinder_mode = selectModeForFramerate(size, options_->framerate.value());

	if (have_raw_stream)
	{
		configuration_->at(raw_stream_num).size = options_->viewfinder_mode.Size();
		configuration_->at(raw_stream_num).pixelFormat = mode_to_pixel_format(options_->viewfinder_mode);
		configuration_->at(raw_stream_num).bufferCount = configuration_->at(0).bufferCount;
	}

	configuration_->transform = options_->transform;

	post_processor_.AdjustConfig("viewfinder", &configuration_->at(0));

	configureDenoise(options_->denoise == "auto" ? "cdn_off" : options_->denoise);
	setupCapture();

	streams_["viewfinder"] = configuration_->at(0).stream();
	if (have_lores_stream)
		streams_["lores"] = configuration_->at(lores_stream_num).stream();
	if (have_raw_stream)
		streams_["raw"] = configuration_->at(raw_stream_num).stream();

	post_processor_.Configure();

	LOG(2, "Viewfinder setup complete");
}

void LibcameraApp::ConfigureStill(unsigned int flags)
{
	LOG(2, "Configuring still capture...");

	// Always request a raw stream as this forces the full resolution capture mode.
	// (options_->mode can override the choice of camera mode, however.)
	StreamRoles stream_roles = { StreamRole::StillCapture, StreamRole::Raw };
	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate still capture configuration");

	// Now we get to override any of the default settings from the options_->
	if (flags & FLAG_STILL_BGR)
		configuration_->at(0).pixelFormat = libcamera::formats::BGR888;
	else if (flags & FLAG_STILL_RGB)
		configuration_->at(0).pixelFormat = libcamera::formats::RGB888;
	else
		configuration_->at(0).pixelFormat = libcamera::formats::YUV420;
	if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_DOUBLE_BUFFER)
		configuration_->at(0).bufferCount = 2;
	else if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_TRIPLE_BUFFER)
		configuration_->at(0).bufferCount = 3;
	else if (options_->buffer_count > 0)
		configuration_->at(0).bufferCount = options_->buffer_count;
	if (options_->width)
		configuration_->at(0).size.width = options_->width;
	if (options_->height)
		configuration_->at(0).size.height = options_->height;
	configuration_->at(0).colorSpace = libcamera::ColorSpace::Sycc;
	configuration_->transform = options_->transform;

	post_processor_.AdjustConfig("still", &configuration_->at(0));

	if (options_->mode.bit_depth)
	{
		configuration_->at(1).size = options_->mode.Size();
		configuration_->at(1).pixelFormat = mode_to_pixel_format(options_->mode);
	}
	configuration_->at(1).bufferCount = configuration_->at(0).bufferCount;

	configureDenoise(options_->denoise == "auto" ? "cdn_hq" : options_->denoise);
	setupCapture();

	streams_["still"] = configuration_->at(0).stream();
	streams_["raw"] = configuration_->at(1).stream();

	post_processor_.Configure();

	LOG(2, "Still capture setup complete");
}

void LibcameraApp::ConfigureVideo(unsigned int flags)
{
	LOG(2, "Configuring video...");

	bool select_mode = options_->framerate && options_->framerate.value() && options_->mode_string.empty();
	bool have_raw_stream = (flags & FLAG_VIDEO_RAW) || options_->mode.bit_depth || select_mode;
	bool have_lores_stream = options_->lores_width && options_->lores_height;
	StreamRoles stream_roles = { StreamRole::VideoRecording };
	int lores_index = 1;
	if (have_raw_stream)
	{
		stream_roles.push_back(StreamRole::Raw);
		lores_index = 2;
	}
	if (have_lores_stream)
		stream_roles.push_back(StreamRole::Viewfinder);
	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate video configuration");

	// Now we get to override any of the default settings from the options_->
	StreamConfiguration &cfg = configuration_->at(0);
	cfg.pixelFormat = libcamera::formats::YUV420;
	cfg.bufferCount = 6; // 6 buffers is better than 4
	if (options_->buffer_count > 0)
		cfg.bufferCount = options_->buffer_count;
	if (options_->width)
		cfg.size.width = options_->width;
	if (options_->height)
		cfg.size.height = options_->height;
	if (flags & FLAG_VIDEO_JPEG_COLOURSPACE)
		cfg.colorSpace = libcamera::ColorSpace::Sycc;
	else if (cfg.size.width >= 1280 || cfg.size.height >= 720)
		cfg.colorSpace = libcamera::ColorSpace::Rec709;
	else
		cfg.colorSpace = libcamera::ColorSpace::Smpte170m;
	configuration_->transform = options_->transform;

	post_processor_.AdjustConfig("video", &configuration_->at(0));

	if (select_mode)
		options_->mode = selectModeForFramerate(cfg.size, options_->framerate.value());

	if (have_raw_stream)
	{
		if (options_->mode.bit_depth)
		{
			configuration_->at(1).size = options_->mode.Size();
			configuration_->at(1).pixelFormat = mode_to_pixel_format(options_->mode);
		}
		else if (!options_->rawfull)
			configuration_->at(1).size = configuration_->at(0).size;
		configuration_->at(1).bufferCount = configuration_->at(0).bufferCount;
	}
	if (have_lores_stream)
	{
		Size lores_size(options_->lores_width, options_->lores_height);
		lores_size.alignDownTo(2, 2);
		if (lores_size.width > configuration_->at(0).size.width ||
			lores_size.height > configuration_->at(0).size.height)
			throw std::runtime_error("Low res image larger than video");
		configuration_->at(lores_index).pixelFormat = libcamera::formats::YUV420;
		configuration_->at(lores_index).size = lores_size;
		configuration_->at(lores_index).bufferCount = configuration_->at(0).bufferCount;
	}
	configuration_->transform = options_->transform;

	configureDenoise(options_->denoise == "auto" ? "cdn_fast" : options_->denoise);
	setupCapture();

	streams_["video"] = configuration_->at(0).stream();
	if (have_raw_stream)
		streams_["raw"] = configuration_->at(1).stream();
	if (have_lores_stream)
		streams_["lores"] = configuration_->at(lores_index).stream();

	post_processor_.Configure();

	LOG(2, "Video setup complete");
}

void LibcameraApp::Teardown()
{
	stopPreview();

	post_processor_.Teardown();

	if (!options_->help)
		LOG(2, "Tearing down requests, buffers and configuration");

	for (auto &iter : mapped_buffers_)
	{
		// assert(iter.first->planes().size() == iter.second.size());
		// for (unsigned i = 0; i < iter.first->planes().size(); i++)
		for (auto &span : iter.second)
			munmap(span.data(), span.size());
	}
	mapped_buffers_.clear();

	delete allocator_;
	allocator_ = nullptr;

	configuration_.reset();

	frame_buffers_.clear();

	streams_.clear();
}

void LibcameraApp::StartCamera()
{
	// This makes all the Request objects that we shall need.
	makeRequests();

	// Build a list of initial controls that we must set in the camera before starting it.
	// We don't overwrite anything the application may have set before calling us.
	if (!controls_.get(controls::ScalerCrop) && options_->roi_width != 0 && options_->roi_height != 0)
	{
		Rectangle sensor_area = *camera_->properties().get(properties::ScalerCropMaximum);
		int x = options_->roi_x * sensor_area.width;
		int y = options_->roi_y * sensor_area.height;
		int w = options_->roi_width * sensor_area.width;
		int h = options_->roi_height * sensor_area.height;
		Rectangle crop(x, y, w, h);
		crop.translateBy(sensor_area.topLeft());
		LOG(2, "Using crop " << crop.toString());
		controls_.set(controls::ScalerCrop, crop);
	}

	if (!controls_.get(controls::AfWindows) && !controls_.get(controls::AfMetering) && options_->afWindow_width != 0 &&
		options_->afWindow_height != 0)
	{
		Rectangle sensor_area = *camera_->properties().get(properties::ScalerCropMaximum);
		int x = options_->afWindow_x * sensor_area.width;
		int y = options_->afWindow_y * sensor_area.height;
		int w = options_->afWindow_width * sensor_area.width;
		int h = options_->afWindow_height * sensor_area.height;
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
		else if (!options_->framerate || options_->framerate.value() > 0)
		{
			int64_t frame_time = 1000000 / options_->framerate.value_or(DEFAULT_FRAMERATE); // in us
			controls_.set(controls::FrameDurationLimits,
						  libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
		}
	}

	if (!controls_.get(controls::ExposureTime) && options_->shutter)
		controls_.set(controls::ExposureTime, options_->shutter);
	if (!controls_.get(controls::AnalogueGain) && options_->gain)
		controls_.set(controls::AnalogueGain, options_->gain);
	if (!controls_.get(controls::AeMeteringMode))
		controls_.set(controls::AeMeteringMode, options_->metering_index);
	if (!controls_.get(controls::AeExposureMode))
		controls_.set(controls::AeExposureMode, options_->exposure_index);
	if (!controls_.get(controls::ExposureValue))
		controls_.set(controls::ExposureValue, options_->ev);
	if (!controls_.get(controls::AwbMode))
		controls_.set(controls::AwbMode, options_->awb_index);
	if (!controls_.get(controls::ColourGains) && options_->awb_gain_r && options_->awb_gain_b)
		controls_.set(controls::ColourGains,
					  libcamera::Span<const float, 2>({ options_->awb_gain_r, options_->awb_gain_b }));
	if (!controls_.get(controls::Brightness))
		controls_.set(controls::Brightness, options_->brightness);
	if (!controls_.get(controls::Contrast))
		controls_.set(controls::Contrast, options_->contrast);
	if (!controls_.get(controls::Saturation))
		controls_.set(controls::Saturation, options_->saturation);
	if (!controls_.get(controls::Sharpness))
		controls_.set(controls::Sharpness, options_->sharpness);

	// AF Controls, where supported and not already set
	if (!controls_.get(controls::AfMode) && camera_->controls().count(&controls::AfMode) > 0)
	{
		int afm = options_->afMode_index;
		if (afm == -1)
		{
			// Choose a default AF mode based on other options
			if (options_->lens_position || options_->set_default_lens_position || options_->af_on_capture)
				afm = controls::AfModeManual;
			else
				afm = camera_->controls().at(&controls::AfMode).max().get<int>();
		}
		controls_.set(controls::AfMode, afm);
	}
	if (!controls_.get(controls::AfRange) && camera_->controls().count(&controls::AfRange) > 0)
		controls_.set(controls::AfRange, options_->afRange_index);
	if (!controls_.get(controls::AfSpeed) && camera_->controls().count(&controls::AfSpeed) > 0)
		controls_.set(controls::AfSpeed, options_->afSpeed_index);

	if (controls_.get(controls::AfMode).value_or(controls::AfModeManual) == controls::AfModeAuto)
	{
		// When starting a viewfinder or video stream in AF "auto" mode,
		// trigger a scan now (but don't move the lens when capturing a still).
		// If an application requires more control over AF triggering, it may
		// override this behaviour with prior settings of AfMode or AfTrigger.
		if (!StillStream() && !controls_.get(controls::AfTrigger))
			controls_.set(controls::AfTrigger, controls::AfTriggerStart);
	}
	else if ((options_->lens_position || options_->set_default_lens_position) &&
			 camera_->controls().count(&controls::LensPosition) > 0 && !controls_.get(controls::LensPosition))
	{
		float f;
		if (options_->lens_position)
			f = options_->lens_position.value();
		else
			f = camera_->controls().at(&controls::LensPosition).def().get<float>();
		LOG(2, "Setting LensPosition: " << f);
		controls_.set(controls::LensPosition, f);
	}

	if (camera_->start(&controls_))
		throw std::runtime_error("failed to start camera");
	controls_.clear();
	camera_started_ = true;
	last_timestamp_ = 0;

	post_processor_.Start();

	camera_->requestCompleted.connect(this, &LibcameraApp::requestComplete);

	for (std::unique_ptr<Request> &request : requests_)
	{
		if (camera_->queueRequest(request.get()) < 0)
			throw std::runtime_error("Failed to queue request");
	}

	LOG(2, "Camera started!");
}

void LibcameraApp::StopCamera()
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
		camera_->requestCompleted.disconnect(this, &LibcameraApp::requestComplete);

	// An application might be holding a CompletedRequest, so queueRequest will get
	// called to delete it later, but we need to know not to try and re-queue it.
	completed_requests_.clear();

	msg_queue_.Clear();

	requests_.clear();

	controls_.clear(); // no need for mutex here

	if (!options_->help)
		LOG(2, "Camera stopped!");
}

LibcameraApp::Msg LibcameraApp::Wait()
{
	return msg_queue_.Wait();
}

void LibcameraApp::queueRequest(CompletedRequest *completed_request)
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

void LibcameraApp::PostMessage(MsgType &t, MsgPayload &p)
{
	msg_queue_.Post(Msg(t, std::move(p)));
}

libcamera::Stream *LibcameraApp::GetStream(std::string const &name, StreamInfo *info) const
{
	auto it = streams_.find(name);
	if (it == streams_.end())
		return nullptr;
	if (info)
		*info = GetStreamInfo(it->second);
	return it->second;
}

libcamera::Stream *LibcameraApp::ViewfinderStream(StreamInfo *info) const
{
	return GetStream("viewfinder", info);
}

libcamera::Stream *LibcameraApp::StillStream(StreamInfo *info) const
{
	return GetStream("still", info);
}

libcamera::Stream *LibcameraApp::RawStream(StreamInfo *info) const
{
	return GetStream("raw", info);
}

libcamera::Stream *LibcameraApp::VideoStream(StreamInfo *info) const
{
	return GetStream("video", info);
}

libcamera::Stream *LibcameraApp::LoresStream(StreamInfo *info) const
{
	return GetStream("lores", info);
}

libcamera::Stream *LibcameraApp::GetMainStream() const
{
	for (auto &p : streams_)
	{
		if (p.first == "viewfinder" || p.first == "still" || p.first == "video")
			return p.second;
	}

	return nullptr;
}

std::vector<libcamera::Span<uint8_t>> LibcameraApp::Mmap(FrameBuffer *buffer) const
{
	auto item = mapped_buffers_.find(buffer);
	if (item == mapped_buffers_.end())
		return {};
	return item->second;
}

void LibcameraApp::ShowPreview(CompletedRequestPtr &completed_request, Stream *stream)
{
	std::lock_guard<std::mutex> lock(preview_item_mutex_);
	if (!preview_item_.stream)
		preview_item_ = PreviewItem(completed_request, stream); // copy the shared_ptr here
	else
		preview_frames_dropped_++;
	preview_cond_var_.notify_one();
}

void LibcameraApp::SetControls(const ControlList &controls)
{
	std::lock_guard<std::mutex> lock(control_mutex_);

	// Add new controls to the stored list. If a control is duplicated,
	// the value in the argument replaces the previously stored value.
	// These controls will be applied to the next StartCamera or request.
	for (const auto &c : controls)
		controls_.set(c.first, c.second);
}

StreamInfo LibcameraApp::GetStreamInfo(Stream const *stream) const
{
	StreamConfiguration const &cfg = stream->configuration();
	StreamInfo info;
	info.width = cfg.size.width;
	info.height = cfg.size.height;
	info.stride = cfg.stride;
	info.pixel_format = stream->configuration().pixelFormat;
	info.colour_space = stream->configuration().colorSpace;
	return info;
}

void LibcameraApp::setupCapture()
{
	// First finish setting up the configuration.

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

	allocator_ = new FrameBufferAllocator(camera_);
	for (StreamConfiguration &config : *configuration_)
	{
		Stream *stream = config.stream();

		if (allocator_->allocate(stream) < 0)
			throw std::runtime_error("failed to allocate capture buffers");

		for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream))
		{
			// "Single plane" buffers appear as multi-plane here, but we can spot them because then
			// planes all share the same fd. We accumulate them so as to mmap the buffer only once.
			size_t buffer_size = 0;
			for (unsigned i = 0; i < buffer->planes().size(); i++)
			{
				const FrameBuffer::Plane &plane = buffer->planes()[i];
				buffer_size += plane.length;
				if (i == buffer->planes().size() - 1 || plane.fd.get() != buffer->planes()[i + 1].fd.get())
				{
					void *memory = mmap(NULL, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
					mapped_buffers_[buffer.get()].push_back(
						libcamera::Span<uint8_t>(static_cast<uint8_t *>(memory), buffer_size));
					buffer_size = 0;
				}
			}
			frame_buffers_[stream].push(buffer.get());
		}
	}
	LOG(2, "Buffers allocated and mapped");

	startPreview();

	// The requests will be made when StartCamera() is called.
}

void LibcameraApp::makeRequests()
{
	auto free_buffers(frame_buffers_);
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

void LibcameraApp::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
	{
		// If the request is cancelled while the camera is still running, it indicates
		// a hardware timeout. Let the application handle this error.
		if (camera_started_)
			msg_queue_.Post(Msg(MsgType::Timeout));

		return;
	}

	CompletedRequest *r = new CompletedRequest(sequence_++, request);
	CompletedRequestPtr payload(r, [this](CompletedRequest *cr) { this->queueRequest(cr); });
	{
		std::lock_guard<std::mutex> lock(completed_requests_mutex_);
		completed_requests_.insert(r);
	}

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

void LibcameraApp::previewDoneCallback(int fd)
{
	std::lock_guard<std::mutex> lock(preview_mutex_);
	auto it = preview_completed_requests_.find(fd);
	if (it == preview_completed_requests_.end())
		throw std::runtime_error("previewDoneCallback: missing fd " + std::to_string(fd));
	preview_completed_requests_.erase(it); // drop shared_ptr reference
}

void LibcameraApp::startPreview()
{
	preview_abort_ = false;
	preview_thread_ = std::thread(&LibcameraApp::previewThread, this);
}

void LibcameraApp::stopPreview()
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

void LibcameraApp::previewThread()
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
		libcamera::Span span = Mmap(buffer)[0];

		// Fill the frame info with the ControlList items and ancillary bits.
		FrameInfo frame_info(item.completed_request->metadata);
		frame_info.fps = item.completed_request->framerate;
		frame_info.sequence = item.completed_request->sequence;

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
		if (!options_->info_text.empty())
		{
			std::string s = frame_info.ToString(options_->info_text);
			preview_->SetInfoText(s);
		}
	}
}

void LibcameraApp::configureDenoise(const std::string &denoise_mode)
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
