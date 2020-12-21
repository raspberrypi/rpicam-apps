/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * yuv.cpp - dummy stills encoder to save uncompressed data
 */

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include "still_options.hpp"

static void yuv420_save(std::vector<void *> const &mem, int w, int h, int stride,
					  std::string const &filename,
					  StillOptions const &options)
{
	if (options.encoding == "yuv420")
	{
		if ((w&1) || (h&1))
			throw std::runtime_error("both width and height must be even");
		if (mem.size() != 1)
			throw std::runtime_error("incorrect number of planes in YUV420 data");
		FILE *fp = fopen(filename.c_str(), "w");
		if (!fp)
			throw std::runtime_error("failed to open file " + filename);
		try
		{
			uint8_t *Y = (uint8_t *)mem[0];
			for (int j = 0; j < h; j++)
			{
				if (fwrite(Y + j * stride, w, 1, fp) != 1)
					throw std::runtime_error("failed to write file " + filename);
			}
			uint8_t *U = Y + stride * h;
			h /= 2, w /= 2, stride /= 2;
			for (int j = 0; j < h; j++)
			{
				if (fwrite(U + j * stride, w, 1, fp) != 1)
					throw std::runtime_error("failed to write file " + filename);
			}
			uint8_t *V = U + stride * h;
			for (int j = 0; j < h; j++)
			{
				if (fwrite(V + j * stride, w, 1, fp) != 1)
					throw std::runtime_error("failed to write file " + filename);
			}
		}
		catch (std::exception const &e)
		{
			fclose(fp);
			throw;
		}
	}
	else
		throw std::runtime_error("output format " + options.encoding + " not supported");
}

static void yuyv_save(std::vector<void *> const &mem, int w, int h, int stride,
					  std::string const &filename,
					  StillOptions const &options)
{
	if (options.encoding == "yuv420")
	{
		if ((w&1) || (h&1))
			throw std::runtime_error("both width and height must be even");
		FILE *fp = fopen(filename.c_str(), "w");
		if (!fp)
			throw std::runtime_error("failed to open file " + filename);
		try
		{
			// We could doubtless do this much quicker. Though starting with
			// YUV420 planar buffer would have been nice.
			std::vector<uint8_t> row(w);
			uint8_t *ptr = (uint8_t *)mem[0];
			for (int j = 0; j < h; j++, ptr += stride)
			{
				for (int i = 0; i < w; i++)
					row[i] = ptr[i << 1];
				if (fwrite(&row[0], w, 1, fp) != 1)
					throw std::runtime_error("failed to write file " + filename);
			}
			ptr = (uint8_t *)mem[0];
			for (int j = 0; j < h; j+=2, ptr += 2*stride)
			{
				for (int i = 0; i < w/2; i++)
					row[i] = ptr[(i << 2) + 1];
				if (fwrite(&row[0], w/2, 1, fp) != 1)
					throw std::runtime_error("failed to write file " + filename);
			}
			ptr = (uint8_t *)mem[0];
			for (int j = 0; j < h; j+=2, ptr += 2*stride)
			{
				for (int i = 0; i < w/2; i++)
					row[i] = ptr[(i << 2) + 3];
				if (fwrite(&row[0], w/2, 1, fp) != 1)
					throw std::runtime_error("failed to write file " + filename);
			}
			fclose(fp);
		}
		catch (std::exception const &e)
		{
			fclose(fp);
			throw;
		}
	}
	else
		throw std::runtime_error("output format " + options.encoding + " not supported");
}

static void rgb_save(std::vector<void *> const &mem, int w, int h, int stride,
					 std::string const &filename,
					 StillOptions const &options)
{
	if (options.encoding != "rgb")
		throw std::runtime_error("encoding should be set to rgb");
	FILE *fp = fopen(filename.c_str(), "w");
	if (!fp)
		throw std::runtime_error("failed to open file " + filename);
	try
	{
		uint8_t *ptr = (uint8_t *)mem[0];
		for (int j = 0; j < h; j++, ptr += stride)
		{
			if (fwrite(ptr, 3*w, 1, fp) != 1)
				throw std::runtime_error("failed to write file " + filename);
		}
		fclose(fp);
	}
	catch (std::exception const &e)
	{
		fclose(fp);
		throw;
	}
}

void yuv_save(std::vector<void *> const &mem, int w, int h, int stride,
			  libcamera::PixelFormat const &pixel_format,
			  std::string const &filename,
			  StillOptions const &options)
{
	if (pixel_format == libcamera::formats::YUYV)
		yuyv_save(mem, w, h, stride, filename, options);
	else if (pixel_format == libcamera::formats::YUV420)
		yuv420_save(mem, w, h, stride, filename, options);
	else if (pixel_format == libcamera::formats::BGR888 || pixel_format == libcamera::formats::RGB888)
		rgb_save(mem, w, h, stride, filename, options);
	else
		throw std::runtime_error("unrecognised YUV/RGB save format");
}
