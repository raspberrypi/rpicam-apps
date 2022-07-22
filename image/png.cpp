/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * png.cpp - Encode image as png and write to file.
 */

#include <cstdio>
#include <string>

#include <libcamera/formats.h>

#include <png.h>

#include "core/still_options.hpp"
#include "core/stream_info.hpp"

void png_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			  std::string const &filename, StillOptions const *options)
{
	if (info.pixel_format != libcamera::formats::BGR888)
		throw std::runtime_error("pixel format for png should be BGR");

	FILE *fp = filename == "-" ? stdout : fopen(filename.c_str(), "wb");
	png_structp png_ptr = NULL;
	png_infop info_ptr = NULL;

	if (fp == NULL)
		throw std::runtime_error("failed to open file " + filename);

	try
	{
		// Open everything up.
		png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
		if (png_ptr == NULL)
			throw std::runtime_error("failed to create png write struct");

		info_ptr = png_create_info_struct(png_ptr);
		if (info_ptr == NULL)
			throw std::runtime_error("failed to create png info struct");

		if (setjmp(png_jmpbuf(png_ptr)))
			throw std::runtime_error("failed to set png error handling");

		// Set image attributes.
		png_set_IHDR(png_ptr, info_ptr, info.width, info.height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
					 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
		// These settings get us most of the compression, but are much faster.
		png_set_filter(png_ptr, 0, PNG_FILTER_AVG);
		png_set_compression_level(png_ptr, 1);

		// Set up the image data.
		png_byte **row_ptrs = (png_byte **)png_malloc(png_ptr, info.height * sizeof(png_byte *));
		png_byte *row = (uint8_t *)mem[0].data();
		for (unsigned int i = 0; i < info.height; i++, row += info.stride)
			row_ptrs[i] = row;

		png_init_io(png_ptr, fp);
		png_set_rows(png_ptr, info_ptr, row_ptrs);
		png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

		long int size = ftell(fp);
		LOG(2, "Wrote PNG file of " << size << " bytes");

		// Free and close everything and we're done.
		png_free(png_ptr, row_ptrs);
		png_destroy_write_struct(&png_ptr, &info_ptr);
		if (fp != stdout)
			fclose(fp);
	}
	catch (std::exception const &e)
	{
		if (png_ptr)
			png_destroy_write_struct(&png_ptr, &info_ptr);
		if (fp && fp != stdout)
			fclose(fp);
		throw;
	}
}
