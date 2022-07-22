/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * bmp.cpp - Encode image as bmp and write to file.
 */

#include <cstdio>
#include <string>

#include <libcamera/formats.h>

#include "core/still_options.hpp"
#include "core/stream_info.hpp"

struct ImageHeader
{
	uint32_t size = sizeof(ImageHeader);
	uint32_t width;
	int32_t height;
	uint16_t planes = 1;
	uint16_t bitcount = 24;
	uint32_t compression = 0;
	uint32_t imagesize = 0;
	uint32_t xpels = 100000;
	uint32_t ypels = 100000;
	uint32_t clrused = 0;
	uint32_t clrimportant = 0;
};
static_assert(sizeof(ImageHeader) == 40, "ImageHeader size wrong");

struct FileHeader
{
	uint16_t dummy; // 2 dummy bytes so that our uint32_ts line up
	uint8_t type1 = 'B';
	uint8_t type2 = 'M';
	uint32_t filesize;
	uint16_t reserved1 = 0;
	uint16_t reserved2 = 0;
	uint32_t offset = sizeof(FileHeader) - 2 + sizeof(ImageHeader);
};
static_assert(sizeof(FileHeader) == 16, "FileHeader size wrong");

void bmp_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			  std::string const &filename, StillOptions const *options)
{
	if (info.pixel_format != libcamera::formats::RGB888)
		throw std::runtime_error("pixel format for bmp should be RGB");

	FILE *fp = filename == "-" ? stdout : fopen(filename.c_str(), "wb");

	if (fp == NULL)
		throw std::runtime_error("failed to open file " + filename);

	try
	{
		unsigned int line = info.width * 3;
		unsigned int pitch = (line + 3) & ~3; // lines are multiples of 4 bytes
		unsigned int pad = pitch - line;
		uint8_t padding[3] = {};
		uint8_t *ptr = (uint8_t *)mem[0].data();

		FileHeader file_header;
		ImageHeader image_header;
		file_header.filesize = file_header.offset + info.height * pitch;
		image_header.width = info.width;
		image_header.height = -info.height; // make image come out the right way up

		// Don't write the file header's 2 dummy bytes
		if (fwrite((uint8_t *)&file_header + 2, sizeof(file_header) - 2, 1, fp) != 1 ||
			fwrite(&image_header, sizeof(image_header), 1, fp) != 1)
			throw std::runtime_error("failed to write BMP file");

		for (unsigned int i = 0; i < info.height; i++, ptr += info.stride)
		{
			if (fwrite(ptr, line, 1, fp) != 1 || (pad != 0 && fwrite(padding, pad, 1, fp) != 1))
				throw std::runtime_error("failed to write BMP file, row " + std::to_string(i));
		}

		LOG(2, "Wrote " << file_header.filesize << " bytes to BMP file");

		if (fp != stdout)
			fclose(fp);
	}
	catch (std::exception const &e)
	{
		if (fp && fp != stdout)
			fclose(fp);
		throw;
	}
}
