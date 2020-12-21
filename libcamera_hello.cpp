/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>

#include "options.hpp"
#include "libcamera_app.hpp"

using namespace std::placeholders;

typedef LibcameraApp<Options> LibcameraHello;
using RequestCompletePayload = LibcameraHello::RequestCompletePayload;
using BufferMap = LibcameraHello::BufferMap;

// The main event loop for the application.

static void event_loop(LibcameraHello &app)
{
	Options const &options = app.options;

	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();
	// When the preview window is done with a set of buffers, queue them back to libcamera.
	app.SetPreviewDoneCallback(std::bind(&LibcameraHello::QueueRequest, &app, _1));
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
	{
		LibcameraHello::Msg msg = app.Wait();
		if (msg.type == LibcameraHello::MsgType::Quit)
			return;
		else if (msg.type != LibcameraHello::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		if (options.verbose)
			std::cout << "Viewfinder frame " << count << std::endl;
		auto now = std::chrono::high_resolution_clock::now();
		if (options.timeout &&
			now - start_time > std::chrono::milliseconds(options.timeout))
			return;

		BufferMap &buffers = std::get<RequestCompletePayload>(msg.payload).buffers;
		app.ShowPreview(buffers, app.ViewfinderStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraHello app;
		if (app.options.Parse(argc, argv))
		{
			if (app.options.verbose)
				app.options.Print();
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
