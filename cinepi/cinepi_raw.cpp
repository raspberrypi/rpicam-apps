/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_raw.cpp - libcamera raw video record app.
 */

#include <chrono>

#include "cinepi_controller.hpp"
#include "dng_encoder.hpp"
#include "output/output.hpp"

using namespace std::placeholders;

// The main even loop for the application.

static void event_loop(CinePIRecorder &app, CinePIController &controller)
{
	controller.start();

	RawOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.ConfigureVideo(CinePIRecorder::FLAG_VIDEO_RAW);
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
	{
		CinePIRecorder::Msg msg = app.Wait();

		if (msg.type == LibcameraApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type != CinePIRecorder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		if (count == 0)
		{
			libcamera::StreamConfiguration const &cfg = app.RawStream()->configuration();
			LOG(1, "Raw stream: " << cfg.size.width << "x" << cfg.size.height << " stride " << cfg.stride << " format "
								  << cfg.pixelFormat.toString());
		}

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

		controller.process(completed_request);

		int trigger = controller.triggerRec();
		if(trigger > 0){
			controller.folderOpen = create_clip_folder(app.GetOptions(), controller.getClipNumber());
		} else if (trigger < 0){
			controller.folderOpen = false;
			app.GetEncoder()->resetFrameCount();
		}
	
		if(controller.isRecording() && controller.folderOpen){
			app.EncodeBuffer(completed_request, app.RawStream(), app.LoresStream());
			std::cout << count << std::endl;
		}

		app.ShowPreview(completed_request, app.VideoStream());        
	}
}

int main(int argc, char *argv[])
{
	try
	{
		CinePIRecorder app;
		CinePIController controller(&app);

		RawOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			options->denoise = "off";
			options->lores_width = 400;
			options->lores_height = 200;
			options->redis = "redis://127.0.0.1:6379/0";
			options->mediaDest = "/media/RAW";

			if (options->verbose >= 2)
				options->Print();

			event_loop(app, controller);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
