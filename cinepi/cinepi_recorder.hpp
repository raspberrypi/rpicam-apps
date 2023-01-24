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
	void StopEncoder() { encoder_.reset(); }

	void LibcameraApp::OpenCamera()
	{
		// Make a preview window.
		preview_ = std::unique_ptr<Preview>(make_preview(options_.get()));
		preview_->SetDoneCallback(std::bind(&LibcameraApp::previewDoneCallback, this, std::placeholders::_1));

		LOG(2, "Opening camera...");

		camera_manager_ = std::make_unique<CameraManager>();
		int ret = camera_manager_->start();
		if (ret)
			throw std::runtime_error("camera manager failed to start, code " + std::to_string(-ret));

		std::vector<std::shared_ptr<libcamera::Camera>> cameras = camera_manager_->cameras();
		// Do not show USB webcams as these are not supported in libcamera-apps!
		auto rem = std::remove_if(cameras.begin(), cameras.end(),
								[](auto &cam) { return cam->id().find("/usb") != std::string::npos; });
		cameras.erase(rem, cameras.end());

		if (cameras.size() == 0)
			throw std::runtime_error("no cameras available");
		if (options_->camera >= cameras.size())
			throw std::runtime_error("selected camera is not available");

		std::string const &cam_id = cameras[options_->camera]->id();
		camera_ = camera_manager_->get(cam_id);
		if (!camera_)
			throw std::runtime_error("failed to find camera " + cam_id);

		if (camera_->acquire())
			throw std::runtime_error("failed to acquire camera " + cam_id);
		camera_acquired_ = true;

		LOG(2, "Acquired camera " << cam_id);

		if (!options_->post_process_file.empty())
			post_processor_.Read(options_->post_process_file);
		// The queue takes over ownership from the post-processor.
		post_processor_.SetCallback(
			[this](CompletedRequestPtr &r) { this->msg_queue_.Post(Msg(MsgType::RequestComplete, std::move(r))); });

		if (options_->framerate)
		{
			std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({ libcamera::StreamRole::Raw });
			const libcamera::StreamFormats &formats = config->at(0).formats();

			// Suppress log messages when enumerating camera modes.
			libcamera::logSetLevel("RPI", "ERROR");
			libcamera::logSetLevel("Camera", "ERROR");

			for (const auto &pix : formats.pixelformats())
			{
				for (const auto &size : formats.sizes(pix))
				{
					config->at(0).size = size;
					config->at(0).pixelFormat = pix;
					config->validate();
					camera_->configure(config.get());
					auto fd_ctrl = camera_->controls().find(&controls::FrameDurationLimits);
					sensor_modes_.emplace_back(size, pix, 1.0e6 / fd_ctrl->second.min().get<int64_t>());
				}
			}

			libcamera::logSetLevel("RPI", "INFO");
			libcamera::logSetLevel("Camera", "INFO");
		}
	};

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
