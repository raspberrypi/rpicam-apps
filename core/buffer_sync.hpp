/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * buffer_sync.hpp - Buffer coherency handling
 */

#pragma once

#include <span>

#include <libcamera/framebuffer.h>

class RPiCamApp;

class BufferWriteSync
{
public:
	BufferWriteSync(RPiCamApp *app, libcamera::FrameBuffer *fb);
	~BufferWriteSync();

	const std::vector<std::span<uint8_t>> &Get() const;

private:
	libcamera::FrameBuffer *fb_;
	std::vector<std::span<uint8_t>> planes_;
};

class BufferReadSync
{
public:
	BufferReadSync(RPiCamApp *app, libcamera::FrameBuffer *fb);
	~BufferReadSync();

	const std::vector<std::span<uint8_t>> &Get() const;

private:
	std::vector<std::span<uint8_t>> planes_;
};
