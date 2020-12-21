/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_encoder.cpp - libcamera video encoding class.
 */

#include "libcamera_app.hpp"
#include "video_options.hpp"
#include "encoder.hpp"

typedef std::function<void(libcamera::Request::BufferMap &, libcamera::Stream *)> EncodeBufferDoneCallback;
typedef std::function<void(void *,size_t, int64_t, bool)> EncodeOutputReadyCallback;

class LibcameraEncoder: public LibcameraApp<VideoOptions>
{
public:
	using BufferMap = libcamera::Request::BufferMap;
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&LibcameraEncoder::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
	// This is the callback when the encoder tells you it's finished with your input buffer.
	void SetEncodeBufferDoneCallback(EncodeBufferDoneCallback callback)
	{
		encode_buffer_done_callback_ = callback;
	}
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback)
	{
		encode_output_ready_callback_ = callback;
	}
	void EncodeBuffer(BufferMap &buffers, Stream *stream)
	{
		assert(encoder_);
		int w, h, stride;
		StreamDimensions(stream, &w, &h, &stride);
		FrameBuffer *buffer = buffers[stream];
		void *mem = Mmap(buffer)[0];
		if (!buffer || !mem)
			throw std::runtime_error("no buffer to encode");
		int64_t timestamp_ns = buffer->metadata().timestamp;
		std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
		encode_buffer_queue_.push(buffers);
		int index = encoder_->EncodeBuffer(buffer->planes()[0].fd.fd(), buffer->planes()[0].length,
										   mem, w, h, stride, timestamp_ns / 1000);
		// Could use index as a reference to this buffer, but we don't seem to need it.
	}
	void StopEncoder()
	{
		encoder_.reset();
	}

protected:
	virtual void createEncoder()
	{
		encoder_ = std::unique_ptr<Encoder>(Encoder::Create(options));
	}
	std::unique_ptr<Encoder> encoder_;

private:
	void encodeBufferDone(int index)
	{
		(void)index; // don't appear to need it if we assume the codec returns them in order
		BufferMap buffers;
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			if (encode_buffer_queue_.empty())
				throw std::runtime_error("no buffer available to return");
			buffers = std::move(encode_buffer_queue_.front());
			encode_buffer_queue_.pop();
		}
		if (encode_buffer_done_callback_)
			encode_buffer_done_callback_(buffers, VideoStream());
	}

	std::queue<BufferMap> encode_buffer_queue_;
	std::mutex encode_buffer_queue_mutex_;
	EncodeBufferDoneCallback encode_buffer_done_callback_;
	EncodeOutputReadyCallback encode_output_ready_callback_;
};
