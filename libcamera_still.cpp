/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_still.cpp - libcamera stills capture app.
 */

#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include <chrono>

#include "libcamera_app.hpp"
#include "still_options.hpp"

using namespace std::placeholders;
using libcamera::Stream;

class LibcameraStillApp : public LibcameraApp
{
public:
	LibcameraStillApp()
		: LibcameraApp(std::make_unique<StillOptions>())
	{
	}

	StillOptions *GetOptions() const
	{
		return static_cast<StillOptions *>(options_.get());
	}
};

// In jpeg.cpp:
void jpeg_save(std::vector<void *> const &mem, int w, int h, int stride,
			   libcamera::PixelFormat const &pixel_format,
			   libcamera::ControlList const &metadata,
			   std::string const &filename,
			   std::string const &cam_name,
			   StillOptions const *options);

// In yuv.cpp:
void yuv_save(std::vector<void *> const &mem, int w, int h, int stride,
			  libcamera::PixelFormat const &pixel_format,
			  std::string const &filename,
			  StillOptions const *options);

// In dng.cpp:
void dng_save(std::vector<void *> const &mem, int w, int h, int stride,
			   libcamera::PixelFormat const &pixel_format,
			   libcamera::ControlList const &metadata,
			   std::string const &filename,
			   std::string const &cam_name,
			   StillOptions const *options);

// In png.cpp:
void png_save(std::vector<void *> const &mem, int w, int h, int stride,
			  libcamera::PixelFormat const &pixel_format,
			  std::string const &filename,
			  StillOptions const *options);

// In bmp.cpp:
void bmp_save(std::vector<void *> const &mem, int w, int h, int stride,
			  libcamera::PixelFormat const &pixel_format,
			  std::string const &filename,
			  StillOptions const *options);

static std::string generate_filename(StillOptions const *options)
{
	char filename[128];
	if (options->datetime)
	{
		std::time_t raw_time;
		std::time(&raw_time);
		char time_string[32];
		std::tm *time_info = std::localtime(&raw_time);
		std::strftime(time_string, sizeof(time_string), "%m%d%H%M%S", time_info);
		snprintf(filename, sizeof(filename), "%s.%s", time_string, options->encoding.c_str());
	}
	else if (options->timestamp)
		snprintf(filename, sizeof(filename), "%u.%s",
				 (unsigned)time(NULL), options->encoding.c_str());
	else
		snprintf(filename, sizeof(filename), options->output.c_str(), options->framestart);
	filename[sizeof(filename)-1] = 0;
	return std::string(filename);
}

static void update_latest_link(std::string const &filename, StillOptions const *options)
{
	// Create a fixed-name link to the most recent output file, if requested.
	if (!options->latest.empty())
	{
		struct stat buf;
		if (stat(options->latest.c_str(), &buf) == 0 && unlink(options->latest.c_str()))
			std::cout << "WARNING: could not delete latest link " << options->latest << std::endl;
		else
		{
			if (symlink(filename.c_str(), options->latest.c_str()))
				std::cout << "WARNING: failed to create latest link " << options->latest << std::endl;
			else if (options->verbose)
				std::cout << "Link " << options->latest << " created" << std::endl;
		}
	}
}

static void save_image(LibcameraStillApp &app, CompletedRequest &payload,
					   Stream *stream, std::string const &filename)
{
	StillOptions const *options = app.GetOptions();
	int w, h, stride;
	app.StreamDimensions(stream, &w, &h, &stride);
	libcamera::PixelFormat const &pixel_format = stream->configuration().pixelFormat;
	std::vector<void *> mem = app.Mmap(payload.buffers[stream]);
	if (stream == app.RawStream())
		dng_save(mem, w, h, stride, pixel_format,
				 payload.metadata, filename, app.CameraId(), options);
	else if (options->encoding == "jpg")
		jpeg_save(mem, w, h, stride, pixel_format,
				  payload.metadata, filename, app.CameraId(), options);
	else if (options->encoding == "png")
		png_save(mem, w, h, stride, pixel_format, filename, options);
	else if (options->encoding == "bmp")
		bmp_save(mem, w, h, stride, pixel_format, filename, options);
	else
		yuv_save(mem, w, h, stride, pixel_format, filename, options);
	if (options->verbose)
		std::cout << "Saved image " << w << " x " << h << " to file " << filename << std::endl;
}

static void save_images(LibcameraStillApp &app, CompletedRequest &payload)
{
	StillOptions *options = app.GetOptions();
	std::string filename = generate_filename(options);
	save_image(app, payload, app.StillStream(), filename);
	update_latest_link(filename, options);
	if (options->raw)
	{
		filename = filename.substr(0, filename.rfind('.')) + ".dng";
		save_image(app, payload, app.RawStream(), filename);
	}
	options->framestart++;
}

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	std::cout << "Received signal " << signal_number << std::endl;
}
static int get_key_or_signal(StillOptions const *options, pollfd p[1])
{
	int key = 0;
	if (options->keypress)
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
	if (options->signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if (signal_received == SIGUSR2)
			key = 'x';
	}
	return key;
}

// The main even loop for the application.

static void event_loop(LibcameraStillApp &app)
{
	StillOptions const *options = app.GetOptions();
	bool output = !options->output.empty() || options->datetime || options->timestamp; // output requested?
	bool keypress = options->keypress || options->signal; // "signal" mode is much like "keypress" mode
	unsigned int still_flags = LibcameraApp::FLAG_STILL_NONE;
	if (options->encoding == "rgb" || options->encoding == "png")
		still_flags |= LibcameraApp::FLAG_STILL_BGR;
	else if (options->encoding == "bmp")
		still_flags |= LibcameraApp::FLAG_STILL_RGB;
	if (options->raw)
		still_flags |= LibcameraApp::FLAG_STILL_RAW;

	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();
	app.SetPreviewDoneCallback(std::bind(&LibcameraApp::QueueRequest, &app, _1));
	auto start_time = std::chrono::high_resolution_clock::now();
	auto timelapse_time = start_time;

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN } };

	for (unsigned int count = 0; ; count++)
	{
		LibcameraApp::Msg msg = app.Wait();
		if (msg.type == LibcameraApp::MsgType::Quit)
			return;
		else if (msg.type != LibcameraApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		auto now = std::chrono::high_resolution_clock::now();
		int key = get_key_or_signal(options, p);
		if (key == 'x'|| key == 'X')
			return;

		// In viewfinder mode, simply run until the timeout. When that happens, switch to
		// capture mode if an output was requested.
		if (app.ViewfinderStream())
		{
			if (options->verbose)
				std::cout << "Viewfinder frame " << count << std::endl;

			bool timed_out = options->timeout &&
				now - start_time > std::chrono::milliseconds(options->timeout);
			bool keypressed = key == '\n';
			bool timelapse_timed_out = options->timelapse &&
				now - timelapse_time > std::chrono::milliseconds(options->timelapse);

			if (timed_out || keypressed || timelapse_timed_out)
			{
				// Trigger a still capture unless:
				if (!output ||                      // we have no output file
					(timed_out && options->timelapse) || // timed out in timelapse mode
					(!keypressed && keypress))      // no key was pressed (in keypress mode)
					return;
				else
				{
					timelapse_time = std::chrono::high_resolution_clock::now();
					app.StopCamera();
					app.Teardown();
					app.ConfigureStill(still_flags);
					app.StartCamera();
				}
			}
			else
			{
				CompletedRequest &completed_request = std::get<CompletedRequest>(msg.payload);
				app.ShowPreview(completed_request, app.ViewfinderStream());
			}
		}
		// In still capture mode, save a jpeg. Go back to viewfinder if in timelapse mode,
		// otherwise quit.
		else if (app.StillStream())
		{
			app.StopCamera();
			std::cout << "Still capture image received" << std::endl;
			save_images(app, std::get<CompletedRequest>(msg.payload));
			if (options->timelapse)
			{
				app.Teardown();
				app.ConfigureViewfinder();
				app.StartCamera();
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
		LibcameraStillApp app;
		StillOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
		return -1;
    }
	return 0;
}
