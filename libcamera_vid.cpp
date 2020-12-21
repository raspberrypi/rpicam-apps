/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_vid.cpp - libcamera video record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include "libcamera_encoder.hpp"
#include "output.hpp"

using namespace std::placeholders;

using RequestCompletePayload = LibcameraEncoder::RequestCompletePayload;

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	std::cout << "Received signal " << signal_number << std::endl;
}
static int get_key_or_signal(VideoOptions const &options, pollfd p[1])
{
	int key = 0;
	if (options.keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			unsigned int len;
			getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options.signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if (signal_received == SIGUSR2)
			key = 'x';
	}
	return key;
}

// The main even loop for the application.

static void event_loop(LibcameraEncoder &app)
{
	VideoOptions const &options = app.options;
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeBufferDoneCallback(std::bind(&LibcameraEncoder::ShowPreview, &app, _1, _2));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.StartEncoder();

	app.OpenCamera();
	app.SetPreviewDoneCallback(std::bind(&LibcameraEncoder::QueueRequest, &app, _1));
	app.ConfigureVideo();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN } };

	for (unsigned int count = 0; ; count++)
	{
		LibcameraEncoder::Msg msg = app.Wait();
		if (msg.type == LibcameraEncoder::MsgType::Quit)
			return;
		else if (msg.type != LibcameraEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		int key = get_key_or_signal(options, p);
		if (key == '\n')
			output->Signal();

		if (options.verbose)
			std::cout << "Viewfinder frame " << count << std::endl;
		auto now = std::chrono::high_resolution_clock::now();
		if ((options.timeout && now - start_time > std::chrono::milliseconds(options.timeout)) ||
			key == 'x' || key == 'X')
		{
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}

		app.EncodeBuffer(std::get<RequestCompletePayload>(msg.payload).buffers, app.VideoStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraEncoder app;
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
