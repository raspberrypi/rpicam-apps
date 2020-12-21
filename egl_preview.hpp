/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * egl_preview.hpp - X/EGL-based preview window.
 */

#pragma once

#include <map>

#include <X11/Xlib.h>

#include <epoxy/gl.h>
#include <epoxy/egl.h>

#include "preview.hpp"

class EglPreview : public Preview
{
public:
	EglPreview(Options const &options);
	~EglPreview();
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, size_t size, int width, int height, int stride) override;
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	virtual void Reset() override;
	// Check if the window manager has closed the preview.
	virtual bool Quit() override;
private:
	struct Buffer
	{
		Buffer() : fd(-1) {}
		int fd;
		size_t size;
		int width;
		int height;
		int stride;
		GLuint texture;
	};
	void makeWindow(char const *name);
	void makeBuffer(int fd, size_t size, int width, int height, int stride, Buffer &buffer);
	::Display *display_;
	EGLDisplay egl_display_;
	Window window_;
	EGLContext egl_context_;
	EGLSurface egl_surface_;
	std::map<int, Buffer> buffers_; // map the DMABUF's fd to the Buffer
	int last_fd_;
	bool first_time_;
	Atom wm_delete_window_;
	// size of preview window
	int x_;
	int y_;
	int width_;
	int height_;
};
