/* Copyright (c) 2024 Osamu Watanabe Takushoku University, Japan. */
/* ht_encoder.hpp - HTJ2K encoder */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <functional>

#include "core/stream_info.hpp"
#include "core/video_options.hpp"

#include "core/options.hpp"

#include "subprojects/kakadujs/src/HTJ2KEncoder.hpp"

#include "simple_tcp.hpp"

uint8_t hotfix_for_mainheader[32] = {
	0xFF,0x4F,0xFF,0x51,0x00,0x2F,0x40,0x00,
	0x00,0x00,0x07,0x80,0x00,0x00,0x04,0x38,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x07,0x80,0x00,0x00,0x04,0x38
};
class HT_Encoder
{
public:
	HT_Encoder(std::vector<uint8_t> &encoded_, const FrameInfo &info, Options const *options) : 
        abortEncode_(false), 
        abortOutput_(false), 
        index_(0),
        enc(encoded_, info)
	{
		output_thread_ = std::thread(&HT_Encoder::outputThread, this);
		for (int i = 0; i < NUM_ENC_THREADS; i++)
			encode_thread_[i] = std::thread(std::bind(&HT_Encoder::encodeThread, this, i));
		LOG(2, "Opened HT_Encoder");
	}
	~HT_Encoder()
	{
		abortEncode_ = true;
		for (int i = 0; i < NUM_ENC_THREADS; i++)
			encode_thread_[i].join();
		abortOutput_ = true;
		output_thread_.join();
		LOG(2, "HT_Encoder closed");
	}
	// Encode the given buffer.
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
	{
		std::lock_guard<std::mutex> lock(encode_mutex_);
		EncodeItem item = { mem, info, timestamp_us, index_++ };
		encode_queue_.push(item);
		encode_cond_var_.notify_all();
	}

private:
	// How many threads to use. Whichever thread is idle will pick up the next frame.
	static const int NUM_ENC_THREADS = 1;

	// These threads do the actual encoding.
	void encodeThread(int num)
	{
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
					if (abortEncode_ && encode_queue_.empty())
					{
						if (frames)
							LOG(2, "Encode " << frames << " frames, average time "
											 << encode_time.count() * 1000 / frames << "ms");
						// jpeg_destroy_compress(&cinfo);
						return;
					}
					if (!encode_queue_.empty())
					{
						encode_item = encode_queue_.front();
						encode_queue_.pop();
						break;
					}
					else
						encode_cond_var_.wait_for(lock, 200ms);
				}
			}

			// Encode the buffer.
			uint8_t *encoded_buffer = nullptr;
			size_t buffer_len = 0;
			auto start_time = std::chrono::high_resolution_clock::now();
			{
				// std::unique_lock<std::mutex> lock(encode_mutex_);
				encodeHTJ2K(encode_item, encoded_buffer, buffer_len);
			}
			encode_time = (std::chrono::high_resolution_clock::now() - start_time);
			// send codestream via TCP connection
			simple_tcp tcp_socket("133.36.41.118", 4001);
			if (!tcp_socket.create_client())
			{
				// std::unique_lock<std::mutex> lock(encode_mutex_);
				tcp_socket.Tx(encoded_buffer, buffer_len);
			}
			// for (int i = 0; i < 32; ++i)
			// {
			// 	printf("%02X ", encoded_buffer[i]);
			// }
			// printf("\n");
			printf("HT codestream size = %ld, time = %f\n", buffer_len, encode_time);
			frames++;
			// Don't return buffers until the output thread as that's where they're
			// in order again.

			// We push this encoded buffer to another thread so that our
			// application can take its time with the data without blocking the
			// encode process.
			OutputItem output_item = { encoded_buffer, buffer_len, encode_item.timestamp_us, encode_item.index };
			std::lock_guard<std::mutex> lock(output_mutex_);
			output_queue_[num].push(output_item);
			output_cond_var_.notify_one();
		}
	}

	// Handle the output buffers in another thread so as not to block the encoders. The
	// application can take its time, after which we return this buffer to the encoder for
	// re-use.
	void outputThread()
	{
		OutputItem item;
		uint64_t index = 0;
		while (true)
		{
			{
				std::unique_lock<std::mutex> lock(output_mutex_);
				while (true)
				{
					using namespace std::chrono_literals;
					// We look for the thread that's completed the frame we want next.
					// If we don't find it, we wait.
					//
					// Must also check for an abort signal, and if set, all queues must
					// be empty. This is done first to ensure all frame callbacks have
					// had a chance to run.
					bool abort = abortOutput_ ? true : false;
					for (auto &q : output_queue_)
					{
						if (abort && !q.empty())
							abort = false;

						if (!q.empty() && q.front().index == index)
						{
							item = q.front();
							q.pop();
							goto got_item;
						}
					}
					if (abort)
						return;

					output_cond_var_.wait_for(lock, 200ms);
				}
			}
		got_item:
        // no need of callback
			// free(item.mem);
			index++;
		}
	}

	bool abortEncode_;
	bool abortOutput_;
	uint64_t index_;

	struct EncodeItem
	{
		void *mem;
		StreamInfo info;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<EncodeItem> encode_queue_;
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];
	void encodeHTJ2K(EncodeItem &item, uint8_t *&encoded_buffer, size_t &buffer_len)
	{
        enc.setSourceImage((uint8_t *)item.mem, item.info.width * item.info.height * 3);
        enc.encode();
        auto out = enc.getEncodedBytes();
        encoded_buffer = out.data();
        buffer_len = out.size();

	}

	struct OutputItem
	{
		void *mem;
		size_t bytes_used;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<OutputItem> output_queue_[NUM_ENC_THREADS];
	std::mutex output_mutex_;
	std::condition_variable output_cond_var_;
	std::thread output_thread_;

    HTJ2KEncoder enc; // encoder instance
};