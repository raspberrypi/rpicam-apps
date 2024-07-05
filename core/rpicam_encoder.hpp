/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_encoder.cpp - libcamera video encoding class.
 */

#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "core/video_options.hpp"

#include "encoder/encoder.hpp"

typedef std::function<void(void *, size_t, int64_t, bool)> EncodeOutputReadyCallback;
typedef std::function<void(libcamera::ControlList &)> MetadataReadyCallback;

class RPiCamEncoder : public RPiCamApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	RPiCamEncoder() : RPiCamApp(std::make_unique<VideoOptions>()) {}

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&RPiCamEncoder::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
	void SetMetadataReadyCallback(MetadataReadyCallback callback) { metadata_ready_callback_ = callback; }
	void EncodeBuffer(CompletedRequestPtr &completed_request, Stream *stream)
	{
		assert(encoder_);
		StreamInfo info = GetStreamInfo(stream);
		FrameBuffer *buffer = completed_request->buffers[stream];
		BufferReadSync r(this, buffer);
		libcamera::Span span = r.Get()[0];
		void *mem = span.data();
		if (!buffer || !mem)
			throw std::runtime_error("no buffer to encode");
		auto ts = completed_request->metadata.get(controls::SensorTimestamp);
		int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer(buffer->planes()[0].fd.get(), span.size(), mem, info, timestamp_ns / 1000);
	}
	VideoOptions *GetOptions() const { return static_cast<VideoOptions *>(options_.get()); }
	void StopEncoder() { encoder_.reset(); }

	void StartRecording()
	{
		if (!is_recording_)
		{
			is_recording_ = true;

			// Generate filename based on current timestamp
			auto now = std::chrono::system_clock::now();
			auto in_time_t = std::chrono::system_clock::to_time_t(now);

			std::stringstream ss;
			ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");

			std::filesystem::path output_dir = "recordings";  // You can change this to your desired directory
			std::filesystem::create_directories(output_dir);  // Ensure the directory exists

			recording_file_ = (output_dir / (ss.str() + ".mp4")).string();

			if (encoder_)
			{
				encoder_->SetOutputFile(recording_file_);
			}

			std::cout << "Started recording to file: " << recording_file_ << std::endl;
			LOG(2, "Started recording to file: " << recording_file_);
		}
	}

	void StopRecording()
	{
		if (is_recording_)
		{
			std::cout << "Recording stopped" << std::endl;
			is_recording_ = false;

			if (encoder_)
			{
				encoder_->ClearOutputFile();
			}
		}
	}

	bool IsRecording() const { return is_recording_; }


protected:
	virtual void createEncoder()
	{
		StreamInfo info;
		VideoStream(&info);
		if (!info.width || !info.height || !info.stride)
			throw std::runtime_error("video steam is not configured");
		encoder_ = std::unique_ptr<Encoder>(Encoder::Create(GetOptions(), info));
	}
	std::unique_ptr<Encoder> encoder_;

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
	bool is_recording_ = false;
	std::string recording_file_;
};
