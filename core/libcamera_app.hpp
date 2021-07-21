/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_app.hpp - base class for libcamera apps.
 */

#pragma once

#include <sys/mman.h>

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <variant>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>

class Options;
class Preview;

namespace controls = libcamera::controls;
namespace properties = libcamera::properties;

struct CompletedRequest
{
	using BufferMap = libcamera::Request::BufferMap;
	using ControlList = libcamera::ControlList;
	CompletedRequest() {}
	CompletedRequest(unsigned int seq, BufferMap const &b, ControlList const &m)
		: sequence(seq), buffers(b), metadata(m)
	{
	}
	unsigned int sequence;
	BufferMap buffers;
	ControlList metadata;
	float framerate;
};

typedef std::function<void(CompletedRequest &)> PreviewDoneCallback;

class LibcameraApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;
	using ControlList = libcamera::ControlList;
	using Request = libcamera::Request;
	using CameraManager = libcamera::CameraManager;
	using Camera = libcamera::Camera;
	using CameraConfiguration = libcamera::CameraConfiguration;
	using FrameBufferAllocator = libcamera::FrameBufferAllocator;
	using StreamRole = libcamera::StreamRole;
	using StreamRoles = libcamera::StreamRoles;
	using PixelFormat = libcamera::PixelFormat;
	using StreamConfiguration = libcamera::StreamConfiguration;
	using BufferMap = Request::BufferMap;
	using Size = libcamera::Size;
	using Rectangle = libcamera::Rectangle;
	struct QuitPayload
	{
	};
	enum class MsgType
	{
		RequestComplete,
		Quit
	};
	typedef std::variant<CompletedRequest, QuitPayload> MsgPayload;
	struct Msg
	{
		Msg(MsgType const &t, MsgPayload const &p) : type(t), payload(p) {}
		MsgType type;
		MsgPayload payload;
	};

	// Some flags that can be used to give hints to the camera configuration.
	static constexpr unsigned int FLAG_STILL_NONE = 0;
	static constexpr unsigned int FLAG_STILL_BGR = 1; // supply BGR images, not YUV
	static constexpr unsigned int FLAG_STILL_RGB = 2; // supply RGB images, not YUV
	static constexpr unsigned int FLAG_STILL_RAW = 4; // request raw image stream
	static constexpr unsigned int FLAG_STILL_DOUBLE_BUFFER = 8; // double-buffer stream
	static constexpr unsigned int FLAG_STILL_TRIPLE_BUFFER = 16; // triple-buffer stream
	static constexpr unsigned int FLAG_STILL_BUFFER_MASK = 24; // mask for buffer flags

	static constexpr unsigned int FLAG_VIDEO_NONE = 0;
	static constexpr unsigned int FLAG_VIDEO_RAW = 1; // request raw image stream

	LibcameraApp(std::unique_ptr<Options> const opts = nullptr);
	virtual ~LibcameraApp();

	Options *GetOptions() const { return options_.get(); }

	std::string const &CameraId() const;
	void OpenCamera();
	void CloseCamera();

	void ConfigureViewfinder();
	void ConfigureStill(unsigned int flags = FLAG_STILL_NONE);
	void ConfigureVideo(unsigned int flags = FLAG_VIDEO_NONE);

	void Teardown();
	void StartCamera();
	void StopCamera();

	Msg Wait();
	void QueueRequest(CompletedRequest const &completed_request);
	void PostMessage(MsgType &t, MsgPayload &p);

	Stream *GetStream(std::string const &name, int *w = nullptr, int *h = nullptr, int *stride = nullptr) const;
	Stream *ViewfinderStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const;
	Stream *StillStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const;
	Stream *RawStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const;
	Stream *VideoStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const;
	Stream *LoresStream(int *w = nullptr, int *h = nullptr, int *stride = nullptr) const;

	std::vector<void *> Mmap(FrameBuffer *buffer) const;

	void SetPreviewDoneCallback(PreviewDoneCallback preview_done_callback);
	void ShowPreview(CompletedRequest &completed_request, Stream *stream);

	void SetControls(ControlList &controls);
	void StreamDimensions(Stream const *stream, int *w, int *h, int *stride) const;

protected:
	std::unique_ptr<Options> options_;

private:
	template <typename T>
	class MessageQueue
	{
	public:
		template <typename U>
		void Post(U &&msg)
		{
			std::unique_lock<std::mutex> lock(mutex_);
			queue_.push(std::forward<U>(msg));
			cond_.notify_one();
		}
		T Wait()
		{
			std::unique_lock<std::mutex> lock(mutex_);
			cond_.wait(lock, [this] { return !queue_.empty(); });
			T msg = std::move(queue_.front());
			queue_.pop();
			return msg;
		}
		void Clear()
		{
			std::unique_lock<std::mutex> lock(mutex_);
			queue_ = {};
		}

	private:
		std::queue<T> queue_;
		std::mutex mutex_;
		std::condition_variable cond_;
	};
	struct PreviewItem
	{
		PreviewItem() : stream(nullptr) {}
		PreviewItem(CompletedRequest &&b, Stream *s) : completed_request(b), stream(s) {}
		PreviewItem &operator=(PreviewItem &&other)
		{
			completed_request = std::move(other.completed_request);
			stream = other.stream;
			other.stream = nullptr;
			return *this;
		}
		CompletedRequest completed_request;
		Stream *stream;
	};

	void setupCapture();
	void makeRequests();
	void requestComplete(Request *request);
	void previewDoneCallback(int fd);
	void previewThread();
	void configureDenoise(const std::string &denoise_mode);

	std::unique_ptr<CameraManager> camera_manager_;
	std::shared_ptr<Camera> camera_;
	bool camera_acquired_ = false;
	std::unique_ptr<CameraConfiguration> configuration_;
	std::map<FrameBuffer *, std::vector<void *>> mapped_buffers_;
	std::map<std::string, Stream *> streams_;
	FrameBufferAllocator *allocator_ = nullptr;
	std::map<Stream *, std::queue<FrameBuffer *>> frame_buffers_;
	std::mutex free_requests_mutex_;
	std::queue<Request *> free_requests_;
	std::vector<std::unique_ptr<Request>> requests_;
	bool camera_started_ = false;
	std::mutex camera_stop_mutex_;
	MessageQueue<Msg> msg_queue_;
	// Related to the preview window.
	std::unique_ptr<Preview> preview_;
	PreviewDoneCallback preview_done_callback_;
	std::map<int, CompletedRequest> preview_completed_requests_;
	std::mutex preview_mutex_;
	std::mutex preview_item_mutex_;
	PreviewItem preview_item_;
	std::condition_variable preview_cond_var_;
	bool preview_abort_ = false;
	uint32_t preview_frames_displayed_ = 0;
	uint32_t preview_frames_dropped_ = 0;
	std::thread preview_thread_;
	// For setting camera controls.
	std::mutex control_mutex_;
	ControlList controls_;
	// Other:
	uint64_t last_timestamp_;
};
