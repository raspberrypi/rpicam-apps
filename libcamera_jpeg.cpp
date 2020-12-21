/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_jpeg.cpp - minimal libcamera jpeg capture app.
 */

#include <chrono>

#include "libcamera_app.hpp"
#include "still_options.hpp"

using namespace std::placeholders;

typedef LibcameraApp<StillOptions> LibcameraJpeg;
using RequestCompletePayload = LibcameraJpeg::RequestCompletePayload;
using BufferMap = LibcameraJpeg::BufferMap;
using libcamera::Stream;

// In jpeg.cpp:
void jpeg_save(std::vector<void *> const &mem, int w, int h, int stride,
			   libcamera::PixelFormat const &pixel_format,
			   libcamera::ControlList const &metadata,
			   std::string const &filename,
			   std::string const &cam_name,
			   StillOptions const &options);

// The main even loop for the application.

static void event_loop(LibcameraJpeg &app)
{
	StillOptions const &options = app.options;
	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();
	app.SetPreviewDoneCallback(std::bind(&LibcameraJpeg::QueueRequest, &app, _1));
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
	{
		LibcameraJpeg::Msg msg = app.Wait();
		if (msg.type == LibcameraJpeg::MsgType::Quit)
			return;
		else if (msg.type != LibcameraJpeg::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		// In viewfinder mode, simply run until the timeout. When that happens, switch to
		// capture mode.
		if (app.ViewfinderStream())
		{
			auto now = std::chrono::high_resolution_clock::now();
			if (options.timeout && now - start_time > std::chrono::milliseconds(options.timeout))
			{
					app.StopCamera();
					app.Teardown();
					app.ConfigureStill();
					app.StartCamera();
			}
			else
			{
				BufferMap &buffers = std::get<RequestCompletePayload>(msg.payload).buffers;
				app.ShowPreview(buffers, app.ViewfinderStream());
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
			RequestCompletePayload &payload = std::get<RequestCompletePayload>(msg.payload);
			std::vector<void *> mem = app.Mmap(payload.buffers[stream]);
			jpeg_save(mem, w, h, stride, stream->configuration().pixelFormat,
					  payload.metadata, app.options.output, app.CameraId(), app.options);
			return;
		}
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraJpeg app;
		if (app.options.Parse(argc, argv))
		{
			if (app.options.verbose)
				app.options.Print();
			if (app.options.output.empty())
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
