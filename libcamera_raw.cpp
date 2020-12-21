/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_raw.cpp - libcamera raw video record app.
 */

#include <chrono>

#include "libcamera_encoder.hpp"
#include "null_encoder.hpp"
#include "output.hpp"

using namespace std::placeholders;

class LibcameraRaw: public LibcameraEncoder
{
protected:
	// Force the use of "null" encoder.
	void createEncoder() { encoder_ = std::unique_ptr<Encoder>(new NullEncoder(options)); }
};

using RequestCompletePayload = LibcameraRaw::RequestCompletePayload;

// The main even loop for the application.

static void event_loop(LibcameraRaw &app)
{
	VideoOptions const &options = app.options;
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeBufferDoneCallback(std::bind(&LibcameraRaw::QueueRequest, &app, _1));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.StartEncoder();

	app.OpenCamera();
	app.ConfigureVideo(LibcameraRaw::FLAG_VIDEO_RAW);
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
	{
		LibcameraRaw::Msg msg = app.Wait();

		if (msg.type != LibcameraRaw::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		if (count == 0)
		{
			libcamera::StreamConfiguration const &cfg = app.RawStream()->configuration();
			std::cout << "Raw stream: " << cfg.size.width << "x" << cfg.size.height << " stride "
					  << cfg.stride << " format " << cfg.pixelFormat.toString() << std::endl;
		}

		if (options.verbose)
			std::cout << "Viewfinder frame " << count << std::endl;
		auto now = std::chrono::high_resolution_clock::now();
		if (options.timeout &&
			now - start_time > std::chrono::milliseconds(options.timeout))
		{
			app.StopCamera();
			app.StopEncoder();
			return;
		}

		app.EncodeBuffer(std::get<RequestCompletePayload>(msg.payload).buffers, app.RawStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraRaw app;
		if (app.options.Parse(argc, argv))
		{
			app.options.nopreview = true;
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
