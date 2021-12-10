/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_detect.cpp - take pictures when objects are detected
 */

// Example: libcamera-detect --post-process-file object_detect_tf.json --lores-width 400 --lores-height 300 -t 0 --object cat -o cat%03d.jpg

#include <chrono>

#include "core/libcamera_app.hpp"
#include "core/still_options.hpp"

#include "image/image.hpp"

#include "post_processing_stages/object_detect.hpp"

struct DetectOptions : public StillOptions
{
	DetectOptions() : StillOptions()
	{
		using namespace boost::program_options;
		options_.add_options()
			("object", value<std::string>(&object), "Name of object to detect")
			("gap", value<unsigned int>(&gap)->default_value(30), "Smallest gap between captures in frames")
			;
	}

	std::string object;
	unsigned int gap;

	virtual void Print() const override
	{
		StillOptions::Print();
		std::cerr << "    object: " << object << std::endl;
		std::cerr << "    gap: " << gap << std::endl;
	}
};

class LibcameraDetectApp : public LibcameraApp
{
public:
	LibcameraDetectApp() : LibcameraApp(std::make_unique<DetectOptions>()) {}
	DetectOptions *GetOptions() const { return static_cast<DetectOptions *>(options_.get()); }
};

// The main even loop for the application.

static void event_loop(LibcameraDetectApp &app)
{
	DetectOptions *options = app.GetOptions();
	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();
	unsigned int last_capture_frame = 0;

	for (unsigned int count = 0;; count++)
	{
		LibcameraApp::Msg msg = app.Wait();
		if (msg.type == LibcameraApp::MsgType::Quit)
			return;

		// In viewfinder mode, simply run until the timeout, but do a capture if the object
		// we're looking for is detected.
		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (app.ViewfinderStream())
		{
			auto now = std::chrono::high_resolution_clock::now();
			if (options->timeout && now - start_time > std::chrono::milliseconds(options->timeout))
				return;

			std::vector<Detection> detections;
			bool detected = completed_request->sequence - last_capture_frame >= options->gap &&
							completed_request->post_process_metadata.Get("object_detect.results", detections) == 0 &&
							std::find_if(detections.begin(), detections.end(), [options](const Detection &d) {
								return d.name.find(options->object) != std::string::npos;
							}) != detections.end();

			app.ShowPreview(completed_request, app.ViewfinderStream());

			if (detected)
			{
				app.StopCamera();
				app.Teardown();
				app.ConfigureStill();
				app.StartCamera();
				std::cerr << options->object << " detected" << std::endl;
			}
		}
		// In still capture mode, save a jpeg and go back to preview.
		else if (app.StillStream())
		{
			app.StopCamera();
			last_capture_frame = completed_request->sequence;

			StreamInfo info;
			libcamera::Stream *stream = app.StillStream(&info);
			const std::vector<libcamera::Span<uint8_t>> mem = app.Mmap(completed_request->buffers[stream]);

			// Make a filename for the output and save it.
			char filename[128];
			snprintf(filename, sizeof(filename), options->output.c_str(), options->framestart);
			filename[sizeof(filename) - 1] = 0;
			options->framestart++;
			std::cerr << "Save image " << filename << std::endl;
			jpeg_save(mem, info, completed_request->metadata, std::string(filename), app.CameraId(), options);

			// Restart camera in preview mode.
			app.Teardown();
			app.ConfigureViewfinder();
			app.StartCamera();
		}
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraDetectApp app;
		DetectOptions *options = app.GetOptions();
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
