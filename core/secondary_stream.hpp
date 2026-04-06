/*
 * secondary_stream.hpp - secondary raw-frame stream over a UNIX domain socket.
 *
 * Copyright (C) 2025 Secluso, Inc.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Additional terms apply; see the NOTICE file in the repository root.
 */

#ifndef SECONDARY_STREAM_HPP
#define SECONDARY_STREAM_HPP

#include "core/rpicam_app.hpp"
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <optional>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <vector>

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "core/video_options.hpp"

#include "encoder/encoder.hpp"

using namespace std::placeholders;
using Stream = libcamera::Stream;
using FrameBuffer = libcamera::FrameBuffer;
#define SOCKET_PATH "/tmp/rpi_raw_frame_socket"

class SecondaryStream
{
public:
	~SecondaryStream();

	void start();
	void stop();
	void pushFrame(libcamera::Span<uint8_t> span);
	bool ready();

private:
	void run();

	std::thread worker_thread;
	std::mutex m;
	std::condition_variable cv;
	std::atomic<bool> running { false };

	std::optional<libcamera::Span<uint8_t>> latest_frame;
	std::chrono::time_point<std::chrono::steady_clock> last_frame_send;
	int32_t output_fps;
};

#endif
