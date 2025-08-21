/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_jpeg.cpp - minimal libcamera jpeg capture app.
 */

#include <chrono>

#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"

#include "image/image.hpp"

using namespace std::placeholders;
using libcamera::Stream;

class RPiCamJpegApp : public RPiCamApp
{
public:
	RPiCamJpegApp()
		: RPiCamApp(std::make_unique<StillOptions>())
	{
	}

	StillOptions *GetOptions() const
	{
		return static_cast<StillOptions *>(RPiCamApp::GetOptions());
	}
};

// The main even loop for the application.

static void event_loop(RPiCamJpegApp &app)
{
	StillOptions const *options = app.GetOptions();
	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	for (;;)
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

		// In viewfinder mode, simply run until the timeout. When that happens, switch to
		// capture mode.
		if (app.ViewfinderStream())
		{
			auto now = std::chrono::high_resolution_clock::now();
			if (options->Get().timeout && (now - start_time) > options->Get().timeout.value)
			{
				app.StopCamera();
				app.Teardown();
				app.ConfigureStill();
				app.StartCamera();
			}
			else
			{
				CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
				app.ShowPreview(completed_request, app.ViewfinderStream());
			}
		}
		// In still capture mode, save a jpeg and quit.
		else if (app.StillStream())
		{
			app.StopCamera();
			LOG(1, "Still capture image received");

			Stream *stream = app.StillStream();
			StreamInfo info = app.GetStreamInfo(stream);
			CompletedRequestPtr &payload = std::get<CompletedRequestPtr>(msg.payload);
			BufferReadSync r(&app, payload->buffers[stream]);
			const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
			jpeg_save(mem, info, payload->metadata, options->Get().output, app.CameraModel(), options);
			return;
		}
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamJpegApp app;
		StillOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->Get().verbose >= 2)
				options->Get().Print();
			if (options->Get().output.empty())
				throw std::runtime_error("output file name required");

			if (options->GetPlatform() == Platform::PISP)
			{
				LOG_ERROR("WARNING: Capture will not make use of temporal denoise");
				LOG_ERROR("         Consider using rpicam-still with the --zsl option for best results, for example:");
				LOG_ERROR("         rpicam-still --zsl -o " << options->Get().output);
			}

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
