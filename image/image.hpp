/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * image.hpp - still image encoder declarations
 */

#pragma once

#include <string>

#include <libcamera/base/span.h>

#include <libcamera/controls.h>

#include "core/stream_info.hpp"

struct StillOptions;

// In jpeg.cpp:
void jpeg_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			   libcamera::ControlList const &metadata, std::string const &filename, std::string const &cam_model,
			   StillOptions const *options);

// In yuv.cpp:
void yuv_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			  std::string const &filename, StillOptions const *options);

// In dng.cpp:
void dng_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			  libcamera::ControlList const &metadata, std::string const &filename, std::string const &cam_model,
			  StillOptions const *options);

// In png.cpp:
void png_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			  std::string const &filename, StillOptions const *options);

// In bmp.cpp:
void bmp_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			  std::string const &filename, StillOptions const *options);
