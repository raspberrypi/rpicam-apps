/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_jpeg.cpp - minimal libcamera jpeg capture app.
 */

#include <chrono>

#include "core/libcamera_app.hpp"
#include "core/still_options.hpp"

using namespace std::placeholders;
using libcamera::Stream;

class LibcameraJpegApp : public LibcameraApp
{
public:
	LibcameraJpegApp()
		: LibcameraApp(std::make_unique<StillOptions>())
	{
	}

	StillOptions *GetOptions() const
	{
		return static_cast<StillOptions *>(options_.get());
	}
};

// In jpeg.cpp:
void jpeg_save(std::vector<libcamera::Span<uint8_t>> const &mem, int w, int h, int stride,
			   libcamera::PixelFormat const &pixel_format, libcamera::ControlList const &metadata,
			   std::string const &filename, std::string const &cam_name, StillOptions const *options);

// The main even loop for the application.

static void event_loop(LibcameraJpegApp &app)
{
	StillOptions const *options = app.GetOptions();
	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();
	app.SetPreviewDoneCallback(std::bind(&LibcameraApp::QueueRequest, &app, _1));
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
	{
		LibcameraApp::Msg msg = app.Wait();
		if (msg.type == LibcameraApp::MsgType::Quit)
			return;
		else if (msg.type != LibcameraApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		// In viewfinder mode, simply run until the timeout. When that happens, switch to
		// capture mode.
		if (app.ViewfinderStream())
		{
			auto now = std::chrono::high_resolution_clock::now();
			if (options->timeout && now - start_time > std::chrono::milliseconds(options->timeout))
			{
				app.StopCamera();
				app.Teardown();
				app.ConfigureStill();
				app.StartCamera();
			}
			else
			{
				CompletedRequest &completed_request = std::get<CompletedRequest>(msg.payload);
				app.ShowPreview(completed_request, app.ViewfinderStream());
			}
		}
		// In still capture mode, save a jpeg and quit.
		else if (app.StillStream())
		{
			app.StopCamera();
			std::cout << "Still capture image received" << std::endl;

			int w, h, stride;
			Stream *stream = app.StillStream();
			app.StreamDimensions(stream, &w, &h, &stride);
			CompletedRequest &payload = std::get<CompletedRequest>(msg.payload);
			const std::vector<libcamera::Span<uint8_t>> mem = app.Mmap(payload.buffers[stream]);
			jpeg_save(mem, w, h, stride, stream->configuration().pixelFormat, payload.metadata, options->output,
					  app.CameraId(), options);
			return;
		}
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraJpegApp app;
		StillOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose)
				options->Print();
			if (options->output.empty())
				throw std::runtime_error("output file name required");

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
