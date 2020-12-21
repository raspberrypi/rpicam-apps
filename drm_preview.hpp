/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * drm_preview.hpp - DRM-based preview window.
 */

#pragma once

#include "preview.hpp"

class DrmPreview : public Preview
{
public:
	DrmPreview(Options const &options);
	~DrmPreview();
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, size_t size, int width, int height, int stride) override;
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	virtual void Reset() override;
private:
	struct Buffer
	{
		Buffer() : fd(-1) {}
		int fd;
		size_t size;
		int width;
		int height;
		int stride;
		uint32_t bo_handle;
		unsigned int fb_handle;
	};
	void makeBuffer(int fd, size_t size, unsigned int width, unsigned int height,
					unsigned int stride, Buffer &buffer);
	void findCrtc();
	void findPlane();
	int drmfd_;
	int conId_;
	uint32_t crtcId_;
	int crtcIdx_;
	uint32_t planeId_;
	unsigned int out_fourcc_;
	unsigned int x_;
	unsigned int y_;
	unsigned int width_;
	unsigned int height_;
	unsigned int screen_width_;
	unsigned int screen_height_;
	std::map<int, Buffer> buffers_; // map the DMABUF's fd to the Buffer
	int last_fd_;
};
