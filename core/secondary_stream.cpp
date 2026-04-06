/*
 * secondary_stream.cpp - secondary raw-frame stream over a UNIX domain socket.
 *
 * Copyright (C) 2025 Secluso, Inc.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Additional terms apply; see the NOTICE file in the repository root.
 */

#include "secondary_stream.hpp"

SecondaryStream::~SecondaryStream()
{
	stop();
}

void SecondaryStream::stop()
{
	if (running.exchange(false))
	{
		cv.notify_one();
		if (worker_thread.joinable())
		{
			worker_thread.join();
		}
	}
}

void SecondaryStream::start()
{
	std::lock_guard<std::mutex> lock(m);
	if (!running.exchange(true))
	{
		if (worker_thread.joinable())
		{
			worker_thread.join(); // Join previous thread before starting a new one
		}
		worker_thread = std::thread(&SecondaryStream::run, this);
	}
}

void SecondaryStream::run()
{
	// Setup message passing logic.
	int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (server_fd == -1)
	{
		perror("Failed to create socket");
		return;
	}

	// Bind to a socket path
	struct sockaddr_un addr;
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
	unlink(SOCKET_PATH); // Remove existing socket file

	if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
	{
		perror("bind");
		close(server_fd);
		return;
	}

	// Listen for connections
	if (listen(server_fd, SOMAXCONN) == -1)
	{
		perror("listen");
		close(server_fd);
		return;
	}

	//TODO: Should we set non-block flags?
	//int flags = fcntl(server_fd, F_GETFL, 0);
	//fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

	// Constructs a polling routine as opposed to blocking in wait for connection. This allows the process to exit peacefully upon termination.
	struct pollfd pfd;
	pfd.fd = server_fd;
	pfd.events = POLLIN;

	int client_fd = -1;

	while (running.load(std::memory_order_seq_cst))
	{
		client_fd = -1; // Reset back to uninitialized in all scenarios.

		int ret = poll(&pfd, 1, 1000); // We time out for one second
		if (ret == -1)
		{
			if (errno == EINTR)
				continue; // Retry if interrupted

			perror("poll");
			break;
		}
		else if (ret == 0)
		{
			if (!running.load(std::memory_order_seq_cst))
			{
				break; // Exit if stop() was called
			}
			continue;
		}

		client_fd = accept(server_fd, nullptr, nullptr);
		if (client_fd == -1)
		{
			if (errno == EINTR)
				continue;

			perror("accept");
			break;
		}

		std::cerr << "Client connected to UNIX domain socket!\n";

		// Allow sender to specify FPS rate they'd like
		char fps_input_buffer;
		if (recv(client_fd, &fps_input_buffer, 1, 0) >= 0)
		{
			output_fps = fps_input_buffer;
			std::cerr << "Client requested " << output_fps << " FPS of raw video in secondary stream!\n";

			while (running.load(std::memory_order_seq_cst))
			{
				CompletedRequestPtr frame;
				{ // Only take the mutex while we fetch the frame.
					std::unique_lock<std::mutex> lock(m);
					if (!latest_frame.has_value())
					{
						if (!running.load(std::memory_order_seq_cst))
						{
							return;
						}

						// Only wake up when we have a new frame to process.
						cv.wait(lock, [this]
								{ return latest_frame.has_value() || !running.load(std::memory_order_seq_cst); });

						if (!running.load(std::memory_order_seq_cst))
						{
							return; // Exit immediately if stopped.
						}
					}

					if (latest_frame)
					{
						libcamera::Span span = std::move(latest_frame.value());
						latest_frame.reset();

						if (span.size() == 0)
						{
							std::cerr << "Warning: Empty frame, skipping transmission.\n";
							continue;
						}

						// Send frame data via socket
						std::cerr << "Sending size: " << span.size() << "\n";
						ssize_t sent = send(client_fd, span.data(), span.size(), 0);
						if (sent == -1)
						{
							perror("send");
							std::cerr << "Client disconnected, waiting for new connection...\n";
							close(client_fd);

							// We disconnected. Allow a future connection by breaking. This'll loop through the outer while loop again.
							break;
						}
					}
				}
			}
		}
		else
		{
			perror("recv");
			std::cerr << "Client failed to send FPS choice...\n";
			if (client_fd != -1)
			{
				close(client_fd);
			}
		}
	}

	// Clean up
	if (client_fd != -1)
	{
		close(client_fd);
	}
	close(server_fd);
	unlink(SOCKET_PATH);
}

/** 
 * Update our latest frame stored in private context.
 * No use for a queue, we only need to send at a low FPS for motion detection. If it can't send it fast enough for some reason,
 * we're better off discarding any lost frames to always get the latest (don't want to fall behind)
**/
void SecondaryStream::pushFrame(libcamera::Span<uint8_t> span)
{
	{
		std::unique_lock<std::mutex> lock(m);
		latest_frame = span;
		last_frame_send = std::chrono::steady_clock::now();
	} // Mutex is released here before notifying

	cv.notify_one(); // Ensures the waiting thread gets notified **after** releasing the mutex
}

// Let the main thread know when we're ready for another thread.
bool SecondaryStream::ready()
{
	if (last_frame_send.time_since_epoch().count() == 0)
	{
		return true; // If no frame has been sent yet, we're ready.
	}

	const auto now = std::chrono::steady_clock::now();
	const auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_send);

	if (output_fps == 0 || diff.count() > (1000 / output_fps))
	{
		return true; // Enough time has elapsed since the last frame.
	}

	return false;
}
