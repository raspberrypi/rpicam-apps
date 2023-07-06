/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * libav_encoder.hpp - libav video encoder.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

extern "C"
{
#include "libavcodec/avcodec.h"
#include "libavdevice/avdevice.h"
#include "libavformat/avformat.h"
#include "libavutil/audio_fifo.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavutil/imgutils.h"
#include "libavutil/timestamp.h"
#include "libavutil/version.h"
#include "libswresample/swresample.h"
}

#include "encoder.hpp"

class LibAvEncoder : public Encoder
{
public:
	LibAvEncoder(VideoOptions const *options, StreamInfo const &info);
	~LibAvEncoder();
	// Encode the given DMABUF.
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us) override;
	void Signal() override;

private:
	void initVideoCodec(VideoOptions const *options, StreamInfo const &info);
	void initAudioInCodec(VideoOptions const *options, StreamInfo const &info);
	void initAudioOutCodec(VideoOptions const *options, StreamInfo const &info);

	void initOutput();
	void deinitOutput();
	void encode(AVPacket *pkt, unsigned int stream_id);

	void videoThread();
	void audioThread();
	void nextSegment(int64_t timestamp_us, int64_t &update_variable);
	static void releaseBuffer(void *opaque, uint8_t *data);

	std::atomic<bool> output_ready_;
	bool abort_video_;
	bool abort_audio_;
	uint64_t video_start_ts_;
	uint64_t audio_samples_;

	std::queue<AVFrame *> frame_queue_;
	std::mutex video_mutex_;
	std::mutex output_mutex_;
	std::condition_variable video_cv_;
	std::thread video_thread_;
	std::thread audio_thread_;

	// The ordering in the enum below must not change!
	enum Context { Video = 0, AudioOut = 1, AudioIn = 2 };
	AVCodecContext *codec_ctx_[3];
	AVStream *stream_[3];
	AVFormatContext *in_fmt_ctx_;
	AVFormatContext *out_fmt_ctx_;

	// Adding variables used to track and create pauses, segments and split
	int64_t segment_start_ts_;
	int segment_num_;
	int64_t virtual_video_ts_;
	int64_t virtual_audio_ts_;
	int64_t previous_video_timestamp_;
	int64_t previous_audio_timestamp_;
	bool feed_encoder_frames_;
	bool previous_feed_value_;

	std::mutex drm_queue_lock_;
	std::queue<std::unique_ptr<AVDRMFrameDescriptor>> drm_frame_queue_;
};
