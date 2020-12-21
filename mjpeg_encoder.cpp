/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>

#include <jpeglib.h>

#include "mjpeg_encoder.hpp"

MjpegEncoder::MjpegEncoder(VideoOptions const &options)
	: Encoder(options), abort_(false), index_(0)
{
	output_thread_ = std::thread(&MjpegEncoder::outputThread, this);
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i] = std::thread(std::bind(&MjpegEncoder::encodeThread, this, i));
	if (options_.verbose)
		std::cout << "Opened MjpegEncoder" << std::endl;
}

MjpegEncoder::~MjpegEncoder()
{
	abort_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i].join();
	output_thread_.join();
	if (options_.verbose)
		std::cout << "MjpegEncoder closed" << std::endl;
}

int MjpegEncoder::EncodeBuffer(int fd, size_t size,
							  void *mem, int width, int height, int stride,
							  int64_t timestamp_us)
{
	EncodeItem item = { mem, width, height, stride, index_, timestamp_us };
	std::lock_guard<std::mutex> lock(encode_mutex_);
	encode_queue_[index_ % NUM_ENC_THREADS].push(item);
	encode_cond_var_.notify_all();
	return index_++;
}

void MjpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item,
							  uint8_t *&encoded_buffer, long unsigned int &buffer_len)
{
	// Copied from YUV420_to_JPEG_fast in jpeg.cpp.
    cinfo.image_width = item.width;
    cinfo.image_height = item.height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

    jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = true;
    jpeg_set_quality(&cinfo, options_.quality, TRUE);
	encoded_buffer = nullptr;
	buffer_len = 0;
    jpeg_mem_dest(&cinfo, &encoded_buffer, &buffer_len);
    jpeg_start_compress(&cinfo, TRUE);

	int stride2 = item.stride / 2;
	uint8_t *Y = (uint8_t *)item.mem;
	uint8_t *U = (uint8_t *)Y + item.stride * item.height;
	uint8_t *V = (uint8_t *)U + stride2 * (item.height / 2);

	JSAMPROW y_rows[16];
	JSAMPROW u_rows[8];
	JSAMPROW v_rows[8];

	int height_align = item.height & ~15;
	while (cinfo.next_scanline < height_align)
	{
		uint8_t *Y_row = Y + cinfo.next_scanline * item.stride;
		for (int i = 0; i < 16; i++, Y_row += item.stride)
			y_rows[i] = Y_row;
		uint8_t *U_row = U + (cinfo.next_scanline / 2) * stride2;
		uint8_t *V_row = V + (cinfo.next_scanline / 2) * stride2;
		for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
			u_rows[i] = U_row, v_rows[i] = V_row;

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}
	if (cinfo.next_scanline < item.height)
	{
		// Raw data has to be written in blocks of 16 rows, so rows beyond the highest
		// multiple of 16 have to be copied to a 16-row sized buffer and then added.
		std::vector<uint8_t> y_pixels(16 * item.stride);
		std::vector<uint8_t> u_pixels(8 * stride2);
		std::vector<uint8_t> v_pixels(8 * stride2);
		memcpy(&y_pixels[0], Y + height_align * item.stride, (item.height & 15) * item.stride);
		memcpy(&u_pixels[1], U + height_align / 2 * stride2, (item.height & 15) / 2 * stride2);
		memcpy(&v_pixels[1], V + height_align / 2 * stride2, (item.height & 15) / 2 * stride2);

		for (int i = 0; i < 16; i++)
			y_rows[i] = &y_pixels[i * item.stride];
		for (int i = 0; i < 8; i++)
			u_rows[i] = &u_pixels[i * stride2], v_rows[i] = &v_pixels[i * stride2];

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}

    jpeg_finish_compress(&cinfo);
}

void MjpegEncoder::encodeThread(int num)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
	std::chrono::duration<double> encode_time(0);
	uint32_t frames = 0;

	EncodeItem encode_item;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(encode_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				if (abort_)
				{
					if (frames && options_.verbose)
						std::cout << "Encode " << frames << " frames, average time " <<
							encode_time.count() * 1000 / frames << "ms" << std::endl;
					jpeg_destroy_compress(&cinfo);
					return;
				}
				if (!encode_queue_[num].empty())
				{
					encode_item = encode_queue_[num].front();
					encode_queue_[num].pop();
					break;
				}
				else
					encode_cond_var_.wait_for(lock, 200ms);
			}
		}

		// Encode the buffer.
		uint8_t *encoded_buffer = nullptr;
		long unsigned int buffer_len = 0;
		auto start_time = std::chrono::high_resolution_clock::now();
		encodeJPEG(cinfo, encode_item, encoded_buffer, buffer_len);
		encode_time += (std::chrono::high_resolution_clock::now() - start_time);
		frames++;
		// Don't return buffers until the output thread as that's where they're
		// in order again.

		// We push this encoded buffer to another thread so that our
		// application can take its time with the data without blocking the
		// encode process.
		OutputItem output_item = { encoded_buffer, buffer_len,
								   encode_item.index, encode_item.timestamp_us };
		std::lock_guard<std::mutex> lock(output_mutex_);
		output_queue_[num].push(output_item);
		output_cond_var_.notify_one();
	}
}

void MjpegEncoder::outputThread()
{
	OutputItem item;
	int num = 0;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(output_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				if (abort_)
					return;
				if (!output_queue_[num].empty())
				{
					item = output_queue_[num].front();
					output_queue_[num].pop();
					break;
				}
				else
					output_cond_var_.wait_for(lock, 200ms);
			}
		}
		input_done_callback_(item.index);

		output_ready_callback_(item.mem, item.bytes_used, item.timestamp_us, true);
		free(item.mem);
		num = (num + 1) % NUM_ENC_THREADS;
	}
}
