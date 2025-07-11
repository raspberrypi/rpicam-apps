/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_still.cpp - libcamera stills capture app.
 */
#include <chrono>
#include <filesystem>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include <chrono>

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"

#include "output/output.hpp"

#include "image/image.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using libcamera::Stream;

namespace fs = std::filesystem;

class RPiCamStillApp : public RPiCamApp
{
public:
	RPiCamStillApp() : RPiCamApp(std::make_unique<StillOptions>()) {}

	StillOptions *GetOptions() const { return static_cast<StillOptions *>(RPiCamApp::GetOptions()); }
};

static std::string generate_filename(StillOptions const *options)
{
	char filename[128];
	std::string folder = options->Get().output; // sometimes "output" is used as a folder name
	if (!folder.empty() && folder.back() != '/')
		folder += "/";
	if (options->Get().datetime)
	{
		std::time_t raw_time;
		std::time(&raw_time);
		char time_string[32];
		std::tm *time_info = std::localtime(&raw_time);
		std::strftime(time_string, sizeof(time_string), "%m%d%H%M%S", time_info);
		snprintf(filename, sizeof(filename), "%s%s.%s", folder.c_str(), time_string, options->Get().encoding.c_str());
	}
	else if (options->Get().timestamp)
		snprintf(filename, sizeof(filename), "%s%u.%s", folder.c_str(), (unsigned)time(NULL),
				 options->Get().encoding.c_str());
	else
		snprintf(filename, sizeof(filename), options->Get().output.c_str(), options->Get().framestart);
	filename[sizeof(filename) - 1] = 0;
	return std::string(filename);
}

static void update_latest_link(std::string const &filename, StillOptions const *options)
{
	// Create a fixed-name link to the most recent output file, if requested.
	if (!options->Get().latest.empty())
	{
		fs::path link { options->Get().latest };
		fs::path target { filename };

		if (fs::exists(link) && !fs::remove(link))
			LOG_ERROR("WARNING: could not delete latest link " << options->Get().latest);
		else
		{
			fs::create_symlink(target, link);
			LOG(2, "Link " << options->Get().latest << " created");
		}
	}
}

static void save_image(RPiCamStillApp &app, CompletedRequestPtr &payload, Stream *stream,
					   std::string const &filename)
{
	StillOptions const *options = app.GetOptions();
	StreamInfo info = app.GetStreamInfo(stream);
	BufferReadSync r(&app, payload->buffers[stream]);
	const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
	if (stream == app.RawStream())
		dng_save(mem, info, payload->metadata, filename, app.CameraModel(), options);
	else if (options->Get().encoding == "jpg")
		jpeg_save(mem, info, payload->metadata, filename, app.CameraModel(), options);
	else if (options->Get().encoding == "png")
		png_save(mem, info, filename, options);
	else if (options->Get().encoding == "bmp")
		bmp_save(mem, info, filename, options);
	else
		yuv_save(mem, info, filename, options);
	LOG(2, "Saved image " << info.width << " x " << info.height << " to file " << filename);
}

static void save_images(RPiCamStillApp &app, CompletedRequestPtr &payload)
{
	StillOptions *options = app.GetOptions();
	std::string filename = generate_filename(options);
	save_image(app, payload, app.StillStream(), filename);
	update_latest_link(filename, options);
	if (options->Get().raw)
	{
		filename = filename.substr(0, filename.rfind('.')) + ".dng";
		save_image(app, payload, app.RawStream(), filename);
	}
	options->Set().framestart++;
	if (options->Get().wrap)
		options->Set().framestart %= options->Get().wrap;
}

static void save_metadata(StillOptions const *options, libcamera::ControlList &metadata)
{
	std::streambuf *buf = std::cout.rdbuf();
	std::ofstream of;
	const std::string &filename = options->Get().metadata;

	if (filename.compare("-"))
	{
		of.open(filename, std::ios::out);
		buf = of.rdbuf();
	}

	write_metadata(buf, options->Get().metadata_format, metadata, true);
}

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
}
static int get_key_or_signal(StillOptions const *options, pollfd p[1])
{
	int key = 0;
	if (options->Get().keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options->Get().signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if (signal_received == SIGUSR2)
			key = 'x';
		signal_received = 0;
	}
	return key;
}

// The main even loop for the application.

static void event_loop(RPiCamStillApp &app)
{
	StillOptions const *options = app.GetOptions();
	// output requested?
	bool output = !options->Get().output.empty() || options->Get().datetime || options->Get().timestamp;
	// "signal" mode is much like "keypress" mode
	bool keypress = options->Get().keypress || options->Get().signal;
	unsigned int still_flags = RPiCamApp::FLAG_STILL_NONE;
	if (options->Get().encoding == "rgb24" || options->Get().encoding == "png")
		still_flags |= RPiCamApp::FLAG_STILL_BGR;
	if (options->Get().encoding == "rgb48")
		still_flags |= RPiCamApp::FLAG_STILL_BGR48;
	else if (options->Get().encoding == "bmp")
		still_flags |= RPiCamApp::FLAG_STILL_RGB;
	if (options->Get().raw)
		still_flags |= RPiCamApp::FLAG_STILL_RAW;

	app.OpenCamera();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	if (options->Get().immediate)
	{
		app.ConfigureStill(still_flags);
		while (keypress)
		{
			int key = get_key_or_signal(options, p);
			if (key == 'x' || key == 'X')
				return;
			else if (key == '\n')
				break;
			std::this_thread::sleep_for(10ms);
		}
	}
	else if (options->Get().zsl)
		app.ConfigureZsl();
	else
		app.ConfigureViewfinder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();
	auto timelapse_time = start_time;
	int timelapse_frames = 0;
	constexpr int TIMELAPSE_MIN_FRAMES = 6; // at least this many preview frames between captures
	bool keypressed = false;
	enum
	{
		AF_WAIT_NONE,
		AF_WAIT_SCANNING,
		AF_WAIT_FINISHED
	} af_wait_state = AF_WAIT_NONE;
	int af_wait_timeout = 0;

	bool want_capture = options->Get().immediate;
	for (unsigned int count = 0;; count++)
	{
		RPiCamApp::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamApp::MsgType::Quit)
			return;
		else if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		auto now = std::chrono::high_resolution_clock::now();
		int key = get_key_or_signal(options, p);
		if (key == 'x' || key == 'X')
			return;
		if (key == '\n')
			keypressed = true;

		// In viewfinder mode, run until the timeout or keypress. When that happens,
		// if the "--autofocus-on-capture" option was set, trigger an AF scan and wait
		// for it to complete. Then switch to capture mode if an output was requested.
		if (app.ViewfinderStream() && !want_capture)
		{
			LOG(2, "Viewfinder frame " << count);
			timelapse_frames++;

			bool timed_out = options->Get().timeout && (now - start_time) > options->Get().timeout.value;
			bool timelapse_timed_out = options->Get().timelapse &&
									   (now - timelapse_time) > options->Get().timelapse.value &&
									   timelapse_frames >= TIMELAPSE_MIN_FRAMES;

			if (af_wait_state != AF_WAIT_NONE)
			{
				FrameInfo fi(completed_request);
				bool scanning = fi.af_state == libcamera::controls::AfStateScanning;
				if (scanning || (af_wait_state == AF_WAIT_SCANNING && ++af_wait_timeout >= 16))
					af_wait_state = AF_WAIT_FINISHED;
				else if (af_wait_state == AF_WAIT_FINISHED)
					want_capture = true;
			}
			else if (timed_out || keypressed || timelapse_timed_out)
			{
				// Trigger a still capture, unless we timed out in timelapse or keypress mode
				if ((timed_out && options->Get().timelapse) || (!keypressed && keypress))
					return;

				keypressed = false;
				if (options->Get().af_on_capture)
				{
					libcamera::ControlList cl;
					cl.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
					cl.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerStart);
					app.SetControls(cl);
					af_wait_state = AF_WAIT_SCANNING;
					af_wait_timeout = 0;
				}
				else
					want_capture = true;
			}
			if (want_capture)
			{
				if (!output)
					return;
				keypressed = false;
				af_wait_state = AF_WAIT_NONE;
				timelapse_time = std::chrono::high_resolution_clock::now();
				if (!options->Get().zsl)
				{
					app.StopCamera();
					app.Teardown();
					app.ConfigureStill(still_flags);
				}
				if (options->Get().af_on_capture)
				{
					libcamera::ControlList cl;
					cl.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
					cl.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerCancel);
					app.SetControls(cl);
				}
				if (!options->Get().zsl)
					app.StartCamera();
			}
			else
				app.ShowPreview(completed_request, app.ViewfinderStream());
		}
		// In still capture mode, save a jpeg. Go back to viewfinder if in timelapse mode,
		// otherwise quit.
		else if (app.StillStream() && want_capture)
		{
			want_capture = false;
			if (!options->Get().zsl)
				app.StopCamera();
			LOG(1, "Still capture image received");
			save_images(app, completed_request);
			if (!options->Get().metadata.empty())
				save_metadata(options, completed_request->metadata);
			timelapse_frames = 0;
			if (!options->Get().immediate &&
				(options->Get().timelapse || options->Get().signal || options->Get().keypress))
			{
				if (!options->Get().zsl)
				{
					app.Teardown();
					app.ConfigureViewfinder();
				}
				if (options->Get().af_on_capture && options->Get().afMode_index == -1)
				{
					libcamera::ControlList cl;
					cl.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
					cl.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerCancel);
					app.SetControls(cl);
				}
				if (!options->Get().zsl)
					app.StartCamera();
				af_wait_state = AF_WAIT_NONE;
			}
			else
				return;
		}
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamStillApp app;
		StillOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
