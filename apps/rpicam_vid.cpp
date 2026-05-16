/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_vid.cpp - libcamera video record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include "apps/control_socket.hpp"
#include "core/rpicam_encoder.hpp"
#include "output/output.hpp"

using namespace std::placeholders;

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
	int key = 0;
	if (signal_received == SIGINT || signal_received == SIGTERM)
		return 'x';
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
		else if ((signal_received == SIGUSR2) || (signal_received == SIGPIPE))
			key = 'x';
		signal_received = 0;
	}
	return key;
}

static int get_colourspace_flags(std::string const &codec)
{
	if (codec == "mjpeg" || codec == "yuv420")
		return RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
	else
		return RPiCamEncoder::FLAG_VIDEO_NONE;
}

// The main even loop for the application.

static void event_loop(RPiCamEncoder &app)
{
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->Get().codec));

	// Query the maximum fps for the negotiated sensor mode from FrameDurationLimits.
	// Must be read after ConfigureVideo() / camera_->configure() so the pipeline
	// has selected the actual sensor mode and the control range is mode-specific.
	// Reading after StartCamera() would return the same value but is unnecessarily late.
	int caps_max_fps = 0;
	{
		auto cameras = app.GetCameras();
		const int camIdx = options->Get().camera;
		if (camIdx >= 0 && camIdx < static_cast<int>(cameras.size()))
		{
			const auto &controls = cameras[camIdx]->controls();
			const auto it = controls.find(&libcamera::controls::FrameDurationLimits);
			if (it != controls.end())
			{
				const int64_t min_duration_us = it->second.min().get<int64_t>();
				if (min_duration_us > 0)
					caps_max_fps = static_cast<int>(1'000'000LL / min_duration_us);
			}
		}
	}

	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Runtime control socket: path derived from camera index (e.g. --camera 1 → /tmp/rpicam-vid1.sock).
	std::string ctrl_sock_path = "/tmp/rpicam-vid" + std::to_string(options->Get().camera) + ".sock";
	ControlSocket ctrl_socket(ctrl_sock_path);
	if (!ctrl_socket.IsValid())
		LOG_ERROR("ControlSocket: failed to initialise – runtime control unavailable");
	const bool ctrl_is_pisp = app.SupportsScalerCrops();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	signal(SIGINT, default_signal_handler);
	signal(SIGTERM, default_signal_handler);
	// SIGPIPE gets raised when trying to write to an already closed socket. This can happen, when
	// you're using TCP to stream to VLC and the user presses the stop button in VLC. Catching the
	// signal to be able to react on it, otherwise the app terminates.
	signal(SIGPIPE, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	for (unsigned int count = 0;; count++)
	{
		RPiCamEncoder::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamEncoder::MsgType::Quit)
			return;
		else if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		int key = get_key_or_signal(options, p);
		if (key == '\n')
			output->Signal();

		// Check the runtime control socket for pending parameter updates.
		ctrl_socket.AcceptConnections();
		{
			// Send camera capabilities once to every new client.
			// Format: "caps:maxfps=<n>,hasaf=<0|1>\n"
			if (ctrl_socket.HasNewClient())
			{
				auto cameras = app.GetCameras();
				const int camIdx = options->Get().camera;
				const bool has_af = (camIdx >= 0 && camIdx < static_cast<int>(cameras.size()))
										? cameras[camIdx]->controls().count(&libcamera::controls::AfMode) > 0
										: false;
				std::string caps =
					"caps:maxfps=" + std::to_string(caps_max_fps) + ",hasaf=" + (has_af ? "1" : "0") + "\n";
				ctrl_socket.SendToClient(caps);
				ctrl_socket.ClearNewClient();
			}
			libcamera::ControlList cl = ctrl_socket.ReadControls(app.GetSensorArea(), ctrl_is_pisp);
			if (!cl.empty())
				app.SetControls(cl);
		}

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->Get().frames && options->Get().timeout &&
					   ((now - start_time) > options->Get().timeout.value);
		bool frameout = options->Get().frames && count >= options->Get().frames;
		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				LOG(1, "Halting: reached timeout of " << options->Get().timeout.get<std::chrono::milliseconds>()
													  << " milliseconds.");
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}
		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (!app.EncodeBuffer(completed_request, app.VideoStream()))
		{
			// Keep advancing our "start time" if we're still waiting to start recording (e.g.
			// waiting for synchronisation with another camera).
			start_time = now;
			count = 0; // reset the "frames encoded" counter too
		}
		app.ShowPreview(completed_request, app.VideoStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamEncoder app;
		VideoOptions *options = app.GetOptions();
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
