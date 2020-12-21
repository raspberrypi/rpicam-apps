/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * preview.hpp - preview window interface
 */

#pragma once

#include <functional>

#include "options.hpp"

class Preview
{
public:
	typedef std::function<void(int fd)> DoneCallback;

	Preview(Options const &options) : options_(options) {}
	virtual ~Preview() {}
	// This is where the application sets the callback it gets whenever the viewfinder
	// is no longer displaying the buffer and it can be safely recycled.
	void SetDoneCallback(DoneCallback callback)
	{
		done_callback_ = callback;
	}
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, size_t size, int width, int height, int stride) = 0;
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	virtual void Reset() = 0;
	// Check if preview window has been shut down.
	virtual bool Quit() { return false; }
protected:
	DoneCallback done_callback_;
	Options options_;
};
