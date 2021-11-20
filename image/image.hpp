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
#include <libcamera/pixel_format.h>

struct StillOptions;

// In jpeg.cpp:
void jpeg_save(std::vector<libcamera::Span<uint8_t>> const &mem, unsigned int w, unsigned int h, unsigned int stride,
			   libcamera::PixelFormat const &pixel_format, libcamera::ControlList const &metadata,
			   std::string const &filename, std::string const &cam_name, StillOptions const *options);

// In yuv.cpp:
void yuv_save(std::vector<libcamera::Span<uint8_t>> const &mem, unsigned int w, unsigned int h, unsigned int stride,
			  libcamera::PixelFormat const &pixel_format, std::string const &filename, StillOptions const *options);

// In dng.cpp:
void dng_save(std::vector<libcamera::Span<uint8_t>> const &mem, unsigned int w, unsigned int h, unsigned int stride,
			  libcamera::PixelFormat const &pixel_format, libcamera::ControlList const &metadata,
			  std::string const &filename, std::string const &cam_name, StillOptions const *options);

// In png.cpp:
void png_save(std::vector<libcamera::Span<uint8_t>> const &mem, unsigned int w, unsigned int h, unsigned int stride,
			  libcamera::PixelFormat const &pixel_format, std::string const &filename, StillOptions const *options);

// In bmp.cpp:
void bmp_save(std::vector<libcamera::Span<uint8_t>> const &mem, unsigned int w, unsigned int h, unsigned int stride,
			  libcamera::PixelFormat const &pixel_format, std::string const &filename, StillOptions const *options);
