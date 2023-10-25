/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020-2021, Raspberry Pi (Trading) Ltd.
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
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <libcamera/base/span.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>

#include "core/buffer_sync.hpp"
#include "core/completed_request.hpp"
#include "core/dma_heaps.hpp"
#include "core/post_processor.hpp"
#include "core/stream_info.hpp"

struct Options;
class Preview;
struct Mode;

namespace controls = libcamera::controls;
namespace properties = libcamera::properties;

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
	using StreamRole = libcamera::StreamRole;
	using StreamRoles = std::vector<libcamera::StreamRole>;
	using PixelFormat = libcamera::PixelFormat;
	using StreamConfiguration = libcamera::StreamConfiguration;
	using BufferMap = Request::BufferMap;
	using Size = libcamera::Size;
	using Rectangle = libcamera::Rectangle;
	enum class MsgType
	{
		RequestComplete,
		Timeout,
		Quit
	};
	typedef std::variant<CompletedRequestPtr> MsgPayload;
	struct Msg
	{
		Msg(MsgType const &t) : type(t) {}
		template <typename T>
		Msg(MsgType const &t, T p) : type(t), payload(std::forward<T>(p))
		{
		}
		MsgType type;
		MsgPayload payload;
	};
	struct SensorMode
	{
		SensorMode()
			: size({}), format({}), fps(0)
		{
		}
		SensorMode(libcamera::Size _size, libcamera::PixelFormat _format, double _fps)
			: size(_size), format(_format), fps(_fps)
		{
		}
		unsigned int depth() const
		{
			// This is a really ugly way of getting the bit depth of the format.
			// But apart from duplicating the massive bayer format table, there is
			// no other way to determine this.
			std::string fmt = format.toString();
			unsigned int mode_depth = fmt.find("8") != std::string::npos ? 8 :
									  fmt.find("10") != std::string::npos ? 10 :
									  fmt.find("12") != std::string::npos ? 12 : 16;
			return mode_depth;
		}
		libcamera::Size size;
		libcamera::PixelFormat format;
		double fps;
		std::string ToString() const
		{
			std::stringstream ss;
			ss << format.toString() << "," << size.toString() << "/" << fps;
			return ss.str();
		}
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
	static constexpr unsigned int FLAG_VIDEO_JPEG_COLOURSPACE = 2; // force JPEG colour space

	LibcameraApp(std::unique_ptr<Options> const opts = nullptr);
	virtual ~LibcameraApp();

	Options *GetOptions() const { return options_.get(); }

	std::string const &CameraId() const;
	std::string CameraModel() const;
	void OpenCamera();
	void CloseCamera();

	void ConfigureViewfinder();
	void ConfigureStill(unsigned int flags = FLAG_STILL_NONE);
	void ConfigureVideo(unsigned int flags = FLAG_VIDEO_NONE);
	void ConfigureZsl(unsigned int still_flags = FLAG_STILL_NONE);

	void Teardown();
	void StartCamera();
	void StopCamera();

	Msg Wait();
	void PostMessage(MsgType &t, MsgPayload &p);

	Stream *GetStream(std::string const &name, StreamInfo *info = nullptr) const;
	Stream *ViewfinderStream(StreamInfo *info = nullptr) const;
	Stream *StillStream(StreamInfo *info = nullptr) const;
	Stream *RawStream(StreamInfo *info = nullptr) const;
	Stream *VideoStream(StreamInfo *info = nullptr) const;
	Stream *LoresStream(StreamInfo *info = nullptr) const;
	Stream *GetMainStream() const;

	const CameraManager *GetCameraManager() const;
	std::vector<std::shared_ptr<libcamera::Camera>> GetCameras()
	{
		return GetCameras(camera_manager_.get());
	}

	void ShowPreview(CompletedRequestPtr &completed_request, Stream *stream);

	void SetControls(const ControlList &controls);
	StreamInfo GetStreamInfo(Stream const *stream) const;

	static unsigned int verbosity;
	static unsigned int GetVerbosity() { return verbosity; }

	static std::vector<std::shared_ptr<libcamera::Camera>> GetCameras(const CameraManager *cm)
	{
		std::vector<std::shared_ptr<libcamera::Camera>> cameras = cm->cameras();
		// Do not show USB webcams as these are not supported in libcamera-apps!
		auto rem = std::remove_if(cameras.begin(), cameras.end(),
								  [](auto &cam) { return cam->id().find("/usb") != std::string::npos; });
		cameras.erase(rem, cameras.end());
		std::sort(cameras.begin(), cameras.end(), [](auto l, auto r) { return l->id() > r->id(); });
		return cameras;
	}

	friend class BufferWriteSync;
	friend class BufferReadSync;
	friend struct Options;

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
		PreviewItem(CompletedRequestPtr &b, Stream *s) : completed_request(b), stream(s) {}
		PreviewItem &operator=(PreviewItem &&other)
		{
			completed_request = std::move(other.completed_request);
			stream = other.stream;
			other.stream = nullptr;
			return *this;
		}
		CompletedRequestPtr completed_request;
		Stream *stream;
	};

	void initCameraManager();
	void setupCapture();
	void makeRequests();
	void queueRequest(CompletedRequest *completed_request);
	void requestComplete(Request *request);
	void previewDoneCallback(int fd);
	void startPreview();
	void stopPreview();
	void previewThread();
	void configureDenoise(const std::string &denoise_mode);
	Mode selectMode(const Mode &mode) const;

	std::unique_ptr<CameraManager> camera_manager_;
	std::vector<std::shared_ptr<libcamera::Camera>> cameras_;
	std::shared_ptr<Camera> camera_;
	bool camera_acquired_ = false;
	std::unique_ptr<CameraConfiguration> configuration_;
	std::map<FrameBuffer *, std::vector<libcamera::Span<uint8_t>>> mapped_buffers_;
	std::map<std::string, Stream *> streams_;
	DmaHeap dma_heap_;
	std::map<Stream *, std::vector<std::unique_ptr<FrameBuffer>>> frame_buffers_;
	std::vector<std::unique_ptr<Request>> requests_;
	std::mutex completed_requests_mutex_;
	std::set<CompletedRequest *> completed_requests_;
	bool camera_started_ = false;
	std::mutex camera_stop_mutex_;
	MessageQueue<Msg> msg_queue_;
	std::vector<SensorMode> sensor_modes_;
	// Related to the preview window.
	std::unique_ptr<Preview> preview_;
	std::map<int, CompletedRequestPtr> preview_completed_requests_;
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
	uint64_t sequence_ = 0;
	PostProcessor post_processor_;
};
