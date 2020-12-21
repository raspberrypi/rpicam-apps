/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * null_encoder.cpp - dummy "do nothing" video encoder.
 */

#include <chrono>
#include <iostream>
#include <stdexcept>

#include "null_encoder.hpp"

NullEncoder::NullEncoder(VideoOptions const &options) :
	abort_(false), input_count_(0), output_count_(0), Encoder(options)
{
	if (options.verbose)
		std::cout << "Opened NullEncoder" << std::endl;
	output_thread_ = std::thread(&NullEncoder::outputThread, this);
}

NullEncoder::~NullEncoder()
{
	abort_ = true;
	output_thread_.join();
	if (options_.verbose)
		std::cout << "NullEncoder closed" << std::endl;
}

// Push the buffer onto the output queue to be "encoded" and returned.
int NullEncoder::EncodeBuffer(int fd, size_t size,
							  void *mem, int width, int height, int stride,
							  int64_t timestamp_us)
{
	std::lock_guard<std::mutex> lock(output_mutex_);
	OutputItem item = { mem, size, timestamp_us };
	output_queue_.push(item);
	output_cond_var_.notify_one();
	return input_count_++;
}

// Realistically we would probably want more of a queue as the caller's number
// of buffers limits the amount of queueing possible here...
void NullEncoder::outputThread()
{
	OutputItem item;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(output_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				if (!output_queue_.empty())
				{
					item = output_queue_.front();
					output_queue_.pop();
					break;
				}
				else
					output_cond_var_.wait_for(lock, 200ms);
				if (abort_)
					return;
			}
		}
		output_ready_callback_(item.mem, item.length, item.timestamp_us, true);
		input_done_callback_(output_count_++);
	}
}
