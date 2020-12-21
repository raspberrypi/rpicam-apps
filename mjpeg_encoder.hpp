/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.hpp - mjpeg video encoder.
 */

#pragma once

#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>

#include "encoder.hpp"

struct jpeg_compress_struct;

class MjpegEncoder : public Encoder
{
public:
	MjpegEncoder(VideoOptions const &options);
	~MjpegEncoder();
	// Encode the given buffer.
	int EncodeBuffer(int fd, size_t size,
					 void *mem, int width, int height, int stride,
					 int64_t timestamp_us) override;

private:
	// How many threads to use. We toss frames at each thread in turn.
	static const int NUM_ENC_THREADS = 4;

	// These threads do the actual encoding.
	void encodeThread(int num);

	// Handle the output buffers in another thread so as not to block the encoders. The
	// application can take its time, after which we return this buffer to the encoder for
	// re-use.
	void outputThread();

	bool abort_;
	int index_;

	struct EncodeItem
	{
		void *mem;
		int width;
		int height;
		int stride;
		int index;
		int64_t timestamp_us;
	};
	std::queue<EncodeItem> encode_queue_[NUM_ENC_THREADS];
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];
	void encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item,
					uint8_t *&encoded_buffer, long unsigned int &buffer_len);

	struct OutputItem
	{
		void *mem;
		size_t bytes_used;
		int index;
		int64_t timestamp_us;
	};
	std::queue<OutputItem> output_queue_[NUM_ENC_THREADS];
	std::mutex output_mutex_;
	std::condition_variable output_cond_var_;
	std::thread output_thread_;
};
