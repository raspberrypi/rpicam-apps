/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_encoder.cpp - libcamera video encoding class.
 */

#include "core/libcamera_app.hpp"
#include "core/stream_info.hpp"
#include "raw_options.hpp"

#include "dng_encoder.hpp"
#include "encoder/encoder.hpp"

typedef std::function<void(void *, size_t, int64_t, bool)> EncodeOutputReadyCallback;
typedef std::function<void(libcamera::ControlList &)> MetadataReadyCallback;

class CinePIRecorder : public LibcameraApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	CinePIRecorder() : LibcameraApp(std::make_unique<RawOptions>()) {}

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&CinePIRecorder::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
	void SetMetadataReadyCallback(MetadataReadyCallback callback) { metadata_ready_callback_ = callback; }
	void EncodeBuffer(CompletedRequestPtr &completed_request, Stream *stream, Stream *lostream)
	{
		assert(encoder_);

		StreamInfo info = GetStreamInfo(stream);
		StreamInfo loinfo = GetStreamInfo(lostream);

		FrameBuffer *buffer = completed_request->buffers[stream];
		FrameBuffer *loBuffer = completed_request->buffers[lostream];

		libcamera::Span span = Mmap(buffer)[0];
		libcamera::Span lospan = Mmap(loBuffer)[0];

		void *mem = span.data();
		void *lomem = lospan.data();

		if (!buffer || !mem)
			throw std::runtime_error("no buffer to encode");

		if (!loBuffer || !lomem)
			throw std::runtime_error("no buffer to encode, thumbnail");
			
		auto ts = completed_request->metadata.get(controls::SensorTimestamp);
		int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer2(buffer->planes()[0].fd.get(), span.size(), mem, info, lospan.size(), lomem, loinfo, timestamp_ns / 1000, completed_request->metadata);
	}
	RawOptions *GetOptions() const { return static_cast<RawOptions *>(options_.get()); }
	DngEncoder *GetEncoder() { return encoder_.get(); }
	void StopEncoder() { encoder_.reset(); }

protected:
	virtual void createEncoder()
	{
		encoder_ = std::unique_ptr<DngEncoder>(new DngEncoder(GetOptions()));
	}
	std::unique_ptr<DngEncoder> encoder_;

private:

	void encodeBufferDone(void *mem)
	{
		// If non-NULL, mem would indicate which buffer has been completed, but
		// currently we're just assuming everything is done in order. (We could
		// handle this by replacing the queue with a vector of <mem, completed_request>
		// pairs.)
		assert(mem == nullptr);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			if (encode_buffer_queue_.empty())
				throw std::runtime_error("no buffer available to return");
			CompletedRequestPtr &completed_request = encode_buffer_queue_.front();
			if (metadata_ready_callback_ && !GetOptions()->metadata.empty())
				metadata_ready_callback_(completed_request->metadata);
			encode_buffer_queue_.pop(); // drop shared_ptr reference
		}
	}

	std::queue<CompletedRequestPtr> encode_buffer_queue_;
	std::mutex encode_buffer_queue_mutex_;
	EncodeOutputReadyCallback encode_output_ready_callback_;
	MetadataReadyCallback metadata_ready_callback_;
};
