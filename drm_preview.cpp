/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * drm_preview.cpp - DRM-based preview window.
 */

#include <drm.h>
#include <drm_mode.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <drm_fourcc.h>

#include "drm_preview.hpp"

#define ERRSTR strerror(errno)

void DrmPreview::findCrtc()
{
	int i;
	drmModeRes *res = drmModeGetResources(drmfd_);
	if (!res) 
		throw std::runtime_error("drmModeGetResources failed: " + std::string(ERRSTR));

	if (res->count_crtcs <= 0)
		throw std::runtime_error("drm: no crts");

	if (!conId_) {
		if (options_.verbose)
			std::cout << "No connector ID specified.  Choosing default from list:" << std::endl;

		for (i = 0; i < res->count_connectors; i++) {
			drmModeConnector *con = drmModeGetConnector(drmfd_, res->connectors[i]);
			drmModeEncoder *enc = NULL;
			drmModeCrtc *crtc = NULL;

			if (con->encoder_id) {
				enc = drmModeGetEncoder(drmfd_, con->encoder_id);
				if (enc->crtc_id) {
					crtc = drmModeGetCrtc(drmfd_, enc->crtc_id);
				}
			}

			if (!conId_ && crtc) {
				conId_ = con->connector_id;
				crtcId_ = crtc->crtc_id;
			}

			if (crtc) {
				screen_width_ = crtc->width;
				screen_height_ = crtc->height;
			}

			if (options_.verbose)
				std::cout << "Connector " << con->connector_id <<
					" (crtc " << (crtc ? crtc->crtc_id : 0) << "): type " <<
					con->connector_type << ", " << (crtc ? crtc->width : 0) << "x" <<
					(crtc ? crtc->height : 0) <<
					(conId_ == (int)con->connector_id ? " (chosen)" : "") << std::endl;
		}

		if (!conId_)
			throw std::runtime_error("No suitable enabled connector found");
	}

	crtcIdx_ = -1;

	for (i = 0; i < res->count_crtcs; ++i) {
		if (crtcId_ == res->crtcs[i]) {
			crtcIdx_ = i;
			break;
		}
	}

	if (crtcIdx_ == -1) {
		drmModeFreeResources(res);
		throw std::runtime_error("drm: CRTC " + std::to_string(crtcId_) + " not found");
	}

	if (res->count_connectors <= 0) {
		drmModeFreeResources(res);
		throw std::runtime_error("drm: no connectors");
	}

	drmModeConnector *c;
	c = drmModeGetConnector(drmfd_, conId_);
	if (!c) {
		drmModeFreeResources(res);
		throw std::runtime_error("drmModeGetConnector failed: " + std::string(ERRSTR));
	}

	if (!c->count_modes) {
		drmModeFreeConnector(c);
		drmModeFreeResources(res);
		throw std::runtime_error("connector supports no mode");
	}

	if (options_.fullscreen || width_ == 0 || height_ == 0)
	{
		drmModeCrtc *crtc = drmModeGetCrtc(drmfd_, crtcId_);
		x_ = crtc->x;
		y_ = crtc->y;
		width_ = crtc->width;
		height_ = crtc->height;
		drmModeFreeCrtc(crtc);
	}
}

void DrmPreview::findPlane()
{
	drmModePlaneResPtr planes;
	drmModePlanePtr plane;
	unsigned int i;
	unsigned int j;

	planes = drmModeGetPlaneResources(drmfd_);
	if (!planes)
		throw std::runtime_error("drmModeGetPlaneResources failed: " + std::string(ERRSTR));

	try
	{
		for (i = 0; i < planes->count_planes; ++i) {
			plane = drmModeGetPlane(drmfd_, planes->planes[i]);
			if (!planes)
				throw std::runtime_error("drmModeGetPlane failed: " + std::string(ERRSTR));

			if (!(plane->possible_crtcs & (1 << crtcIdx_))) {
				drmModeFreePlane(plane);
				continue;
			}

			for (j = 0; j < plane->count_formats; ++j) {
				if (plane->formats[j] == out_fourcc_) {
					break;
				}
			}

			if (j == plane->count_formats) {
				drmModeFreePlane(plane);
				continue;
			}

			planeId_ = plane->plane_id;
			drmModeFreePlane(plane);
			break;
		}
	}
	catch (std::exception const &e)
	{
		drmModeFreePlaneResources(planes);
		throw;
	}

	drmModeFreePlaneResources(planes);
}

DrmPreview::DrmPreview(Options const &options) : last_fd_(-1), Preview(options)
{
	drmfd_ = drmOpen("vc4", NULL);
	if (drmfd_ < 0)
		throw std::runtime_error("drmOpen failed: " + std::string(ERRSTR));

	x_ = options_.preview_x;
	y_ = options_.preview_y;
	width_ = options.preview_width;
	height_ = options.preview_height;
	screen_width_ = 0;
	screen_height_ = 0;

	try
	{
		conId_ = 0;
		findCrtc();
		out_fourcc_ = DRM_FORMAT_YUV420;
		findPlane();
	}
	catch (std::exception const &e)
	{
		close(drmfd_);
		throw;
	}

	// Default behaviour here is to go fullscreen.
	if (options_.fullscreen || width_ == 0 || height_ == 0 ||
		x_ + width_ > screen_width_ || y_ + height_ > screen_height_)
	{
		x_ = y_ = 0;
		width_ = screen_width_;
		height_ = screen_height_;
	}
}

DrmPreview::~DrmPreview()
{
	close(drmfd_);
}

void DrmPreview::makeBuffer(int fd, size_t size, unsigned int width, unsigned int height,
							unsigned int stride, Buffer &buffer)
{
	buffer.fd = fd;
	buffer.size = size;
	buffer.width = width;
	buffer.height = height;
	buffer.stride = stride;

	if (drmPrimeFDToHandle(drmfd_, fd, &buffer.bo_handle))
		throw std::runtime_error("drmPrimeFDToHandle failed for fd " + std::to_string(fd));

	uint32_t offsets[4] = { 0, stride * height, stride * height + (stride / 2) * (height / 2) };
	uint32_t pitches[4] = { stride, stride / 2, stride / 2 };
	uint32_t bo_handles[4] = { buffer.bo_handle, buffer.bo_handle, buffer.bo_handle };
	
	if (drmModeAddFB2(drmfd_, width, height, out_fourcc_,
					  bo_handles, pitches, offsets, &buffer.fb_handle, 0))
		throw std::runtime_error("drmModeAddFB2 failed: " + std::string(ERRSTR));
}


void DrmPreview::Show(int fd, size_t size, int width, int height, int stride)
{
	Buffer &buffer = buffers_[fd];
	if (buffer.fd == -1)
		makeBuffer(fd, size, width, height, stride, buffer);

	unsigned int x_off = 0, y_off = 0;
	unsigned int w = width_, h = height_;
	if (width * height_ > width_ * height)
		h = width_ * height / width, y_off = (height_ - h ) / 2;
	else
		w = height_ * width / height, x_off = (width_ - w) / 2;

	if (drmModeSetPlane(drmfd_, planeId_, crtcId_,
                        buffer.fb_handle, 0,
                        x_off + x_, y_off + y_, w, h,
                        0, 0,
                        buffer.width << 16,
                        buffer.height << 16))
		throw std::runtime_error("drmModeSetPlane failed: " + std::string(ERRSTR));
   if (last_fd_ >= 0)
	   done_callback_(last_fd_);
   last_fd_ = fd;
}

void DrmPreview::Reset()
{
	for (auto &it : buffers_)
		drmModeRmFB(drmfd_, it.second.fb_handle);
	buffers_.clear();
	last_fd_ = -1;
}
