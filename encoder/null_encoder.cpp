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

NullEncoder::NullEncoder(VideoOptions const *options) : Encoder(options), abort_(false)
{
	LOG(2, "Opened NullEncoder");
	output_thread_ = std::thread(&NullEncoder::outputThread, this);
}

NullEncoder::~NullEncoder()
{
	abort_ = true;
	output_thread_.join();
	LOG(2, "NullEncoder closed");
}

// Push the buffer onto the output queue to be "encoded" and returned.
void NullEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
{
	std::lock_guard<std::mutex> lock(output_mutex_);
	OutputItem item = { mem, size, timestamp_us };
	output_queue_.push(item);
	output_cond_var_.notify_one();
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
		// Ensure the input done callback happens before the output ready callback.
		// This is needed as the metadata queue gets pushed in the former, and popped
		// in the latter.
		input_done_callback_(nullptr);
		output_ready_callback_(item.mem, item.length, item.timestamp_us, true);
	}
}
