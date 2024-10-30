/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>

#include "core/rpicam_app.hpp"
#include "core/options.hpp"
#include "post_processing_stages/object_detect.hpp"
#include "subprojects/kakadujs/src/HTJ2KEncoder.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"


using namespace std::placeholders;

// The main event loop for the application.

static void event_loop(RPiCamApp &app)
{
	Options const *options = app.GetOptions();

	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();

	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
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

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		if (options->timeout && (now - start_time) > options->timeout.value)
			return;

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		// We commented out below because preview window only supports YUV420
		// app.ShowPreview(completed_request, app.ViewfinderStream());
		
		StreamInfo *info;
		libcamera::Stream *stream = app.ViewfinderStream(info);
		
		BufferReadSync w(&app, completed_request->buffers[stream]);
		libcamera::Span<uint8_t> buffer = w.Get()[0];
		uint8_t *ptr = (uint8_t *)buffer.data();
		cv::Mat frame = cv::Mat(info->height, info->width, CV_8UC3, ptr);
		cv::imshow("Dectention results", frame);
		cv::pollKey();
		
		std::vector<Detection> objects;
		completed_request->post_process_metadata.Get("object_detect.results", objects);
		if (objects.size()) {
			
		}
		
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

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
