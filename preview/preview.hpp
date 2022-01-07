/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * preview.hpp - preview window interface
 */

#pragma once

#include <functional>
#include <string>

#include <libcamera/base/span.h>

#include "core/stream_info.hpp"

struct Options;

class Preview
{
public:
	typedef std::function<void(int fd)> DoneCallback;

	Preview(Options const *options) : options_(options) {}
	virtual ~Preview() {}
	// This is where the application sets the callback it gets whenever the viewfinder
	// is no longer displaying the buffer and it can be safely recycled.
	void SetDoneCallback(DoneCallback callback) { done_callback_ = callback; }
	virtual void SetInfoText(const std::string &text) {}
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info) = 0;
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	virtual void Reset() = 0;
	// Check if preview window has been shut down.
	virtual bool Quit() { return false; }
	// Return the maximum image size allowed.
	virtual void MaxImageSize(unsigned int &w, unsigned int &h) const = 0;

protected:
	DoneCallback done_callback_;
	Options const *options_;
};

Preview *make_preview(Options const *options);
