/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * buffer_sync.cpp - Buffer coherency handling
 */

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "core/buffer_sync.hpp"
#include "core/rpicam_app.hpp"
#include "core/logging.hpp"

BufferWriteSync::BufferWriteSync(RPiCamApp *app, libcamera::FrameBuffer *fb)
	: fb_(fb)
{
	struct dma_buf_sync dma_sync {};
	dma_sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;

	auto it = app->mapped_buffers_.find(fb_);
	if (it == app->mapped_buffers_.end())
	{
		LOG_ERROR("failed to find buffer in BufferWriteSync");
		return;
	}

	int ret = ::ioctl(fb_->planes()[0].fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
	if (ret)
	{
		LOG_ERROR("failed to lock-sync-write dma buf");
		return;
	}

	planes_ = it->second;
}

BufferWriteSync::~BufferWriteSync()
{
	struct dma_buf_sync dma_sync {};
	dma_sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;

	int ret = ::ioctl(fb_->planes()[0].fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
	if (ret)
		LOG_ERROR("failed to unlock-sync-write dma buf");
}

const std::vector<libcamera::Span<uint8_t>> &BufferWriteSync::Get() const
{
	return planes_;
}

BufferReadSync::BufferReadSync(RPiCamApp *app, libcamera::FrameBuffer *fb)
{
	auto it = app->mapped_buffers_.find(fb);
	if (it == app->mapped_buffers_.end())
	{
		LOG_ERROR("failed to find buffer in BufferReadSync");
		return;
	}

	// DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ happens when the request completes,
	// so nothing to do here but cache the planes map.
	planes_ = it->second;
}

BufferReadSync::~BufferReadSync()
{
	// DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ happens when we resend the buffer
	// in the next request, so nothing to do here.
}

const std::vector<libcamera::Span<uint8_t>> &BufferReadSync::Get() const
{
	return planes_;
}
