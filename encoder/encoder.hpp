/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * encoder.hpp - Video encoder class.
 */

#pragma once

#include <functional>

#include "core/stream_info.hpp"
#include "core/video_options.hpp"

typedef std::function<void(void *)> InputDoneCallback;
typedef std::function<void(void *, size_t, int64_t, bool)> OutputReadyCallback;

class Encoder
{
public:
	static Encoder *Create(VideoOptions *options, StreamInfo const &info);

	Encoder(VideoOptions const *options) : options_(options) {}
	virtual ~Encoder() {}
	// This is where the application sets the callback it gets whenever the encoder
	// has finished with an input buffer, so the application can re-use it.
	void SetInputDoneCallback(InputDoneCallback callback) { input_done_callback_ = callback; }
	// This callback is how the application is told that an encoded buffer is
	// available. The application may not hang on to the memory once it returns
	// (but the callback is already running in its own thread).
	void SetOutputReadyCallback(OutputReadyCallback callback) { output_ready_callback_ = callback; }
	// Encode the given buffer. The buffer is specified both by an fd and size
	// describing a DMABUF, and by a mmapped userland pointer.
	virtual void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us) = 0;

protected:
	InputDoneCallback input_done_callback_;
	OutputReadyCallback output_ready_callback_;
	VideoOptions const *options_;
};
