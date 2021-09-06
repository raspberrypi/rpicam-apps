/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * jpeg.cpp - Encode image as jpeg and write to file.
 */

#include <cstdio>
#include <cstring>

#include <algorithm>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include <jpeglib.h>
#include <libexif/exif-data.h>

#include "core/still_options.hpp"

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif

using namespace libcamera;

typedef int (*ExifReadFunction)(char const *, unsigned char *);

static int exif_read_short(char const *str, unsigned char *mem);
static int exif_read_sshort(char const *str, unsigned char *mem);
static int exif_read_long(char const *str, unsigned char *mem);
static int exif_read_slong(char const *str, unsigned char *mem);
static int exif_read_rational(char const *str, unsigned char *mem);
static int exif_read_srational(char const *str, unsigned char *mem);

static ExifEntry *exif_create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag);
static void exif_set_string(ExifEntry *entry, char const *s);
static void exif_read_tag(ExifData *exif, char const *str);

static const ExifByteOrder exif_byte_order = EXIF_BYTE_ORDER_INTEL;
static const unsigned int exif_image_offset = 20; // offset of image in JPEG buffer
static const unsigned char exif_header[] = { 0xff, 0xd8, 0xff, 0xe1 };

struct ExifException
{
	ExifFormat format;
	unsigned int components; // can be zero for "variable/unknown"
};

// libexif knows the formats of many tags, but not all (I mean, why not?!?).
// Exceptions can be listed here.
static std::map<ExifTag, ExifException> exif_exceptions =
{
	{ EXIF_TAG_YCBCR_COEFFICIENTS, { EXIF_FORMAT_RATIONAL, 3 } },
};

static std::map<std::string, ExifIfd> exif_ifd_map =
{
	{ "EXIF", EXIF_IFD_EXIF },
	{ "IFD0", EXIF_IFD_0 },
	{ "IFD1", EXIF_IFD_1 },
	{ "EINT", EXIF_IFD_INTEROPERABILITY },
	{ "GPS",  EXIF_IFD_GPS }
};

static ExifReadFunction const exif_read_functions[] =
{
	// Same order as ExifFormat enum.
	nullptr, // dummy
	nullptr, // byte
	nullptr, // ascii
	exif_read_short,
	exif_read_long,
	exif_read_rational,
	nullptr, // sbyte
	nullptr, // undefined
	exif_read_sshort,
	exif_read_slong,
	exif_read_srational
};


int exif_read_short(char const *str, unsigned char *mem)
{
	unsigned short value;
	int n;
	if (sscanf(str, "%hu%n", &value, &n) != 1)
		throw std::runtime_error("failed to read EXIF unsigned short");
	exif_set_short(mem, exif_byte_order, value);
	return n;
}

int exif_read_sshort(char const *str, unsigned char *mem)
{
	short value;
	int n;
	if (sscanf(str, "%hd%n", &value, &n) != 1)
		throw std::runtime_error("failed to read EXIF signed short");
	exif_set_sshort(mem, exif_byte_order, value);
	return n;
}

int exif_read_long(char const *str, unsigned char *mem)
{
	uint32_t value;
	int n;
	if (sscanf(str, "%u%n", &value, &n) != 1)
		throw std::runtime_error("failed to read EXIF unsigned short");
	exif_set_long(mem, exif_byte_order, value);
	return n;
}

int exif_read_slong(char const *str, unsigned char *mem)
{
	int32_t value;
	int n;
	if (sscanf(str, "%d%n", &value, &n) != 1)
		throw std::runtime_error("failed to read EXIF signed short");
	exif_set_slong(mem, exif_byte_order, value);
	return n;
}

int exif_read_rational(char const *str, unsigned char *mem)
{
	uint32_t num, denom;
	int n;
	if (sscanf(str, "%u/%u%n", &num, &denom, &n) != 2)
		throw std::runtime_error("failed to read EXIF unsigned rational");
	exif_set_rational(mem, exif_byte_order, { num, denom });
	return n;
}

int exif_read_srational(char const *str, unsigned char *mem)
{
	int32_t num, denom;
	int n;
	if (sscanf(str, "%d/%d%n", &num, &denom, &n) != 2)
		throw std::runtime_error("failed to read EXIF signed rational");
	exif_set_srational(mem, exif_byte_order, { num, denom });
	return n;
}

ExifEntry *exif_create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
	ExifEntry *entry = exif_content_get_entry(exif->ifd[ifd], tag);
	if (entry)
		return entry;
	entry = exif_entry_new();
	if (!entry)
		throw std::runtime_error("failed to allocate EXIF entry");
	entry->tag = tag;
	exif_content_add_entry(exif->ifd[ifd], entry);
	exif_entry_initialize(entry, entry->tag);
	exif_entry_unref(entry);
	return entry;
}

void exif_set_string(ExifEntry *entry, char const *s)
{
	if (entry->data)
		free(entry->data);
	entry->size = entry->components = strlen(s);
	entry->data = (unsigned char *)strdup(s);
	if (!entry->data)
		throw std::runtime_error("failed to copy exif string");
	entry->format = EXIF_FORMAT_ASCII;
}

void exif_read_tag(ExifData *exif, char const *str)
{
	// Fetch and check the IFD and tag are valid.

	char ifd_name[5];
	char tag_name[128];
	int bytes_consumed;
	if (sscanf(str, "%4[^.].%127[^=]=%n", ifd_name, tag_name, &bytes_consumed) != 2)
		throw std::runtime_error("failed to read EXIF IFD and tag");
	if (exif_ifd_map.count(std::string(ifd_name)) == 0)
		throw std::runtime_error("bad IFD name " + std::string(ifd_name));
	ExifIfd ifd = exif_ifd_map[ifd_name];
	std::string tag_string(tag_name);
	ExifTag tag = exif_tag_from_name(tag_name);
	if (tag == 0)
	{
		std::cout << "WARNING: no EXIF tag " << tag_name << " found - ignoring" << std::endl;
		return;
	}

	// Make an EXIF entry, trying to figure out the correct details and format.

	ExifEntry *entry = exif_create_tag(exif, ifd, tag);
	if (!entry)
		throw std::runtime_error("failed to create entry for EXIF tag " + tag_string + ", please try without this tag");
	if (entry->format == 0)
	{
		std::cout << "WARNING: format for EXIF tag " << tag_name << " unknown - ignoring" << std::endl;
		return;
	}
	if (entry->format == EXIF_FORMAT_UNDEFINED)
	{
		if (exif_exceptions.count(tag))
		{
			ExifException const &exif_exception = exif_exceptions[tag];
			entry->format = exif_exception.format;
			entry->components = exif_exception.components;
		}
		else
		{
			std::cout << "WARNING: libexif format for tag " << tag_name << " undefined - treating as ASCII"
					  << std::endl;
			entry->format = EXIF_FORMAT_ASCII;
		}
	}

	// Finally, read the information into the entry.

	if (entry->format == EXIF_FORMAT_ASCII)
	{
		exif_set_string(entry, str + bytes_consumed);
		return;
	}
	size_t item_size = exif_format_get_size(entry->format);
	if (entry->size == 0 || entry->components == 0 || entry->data == nullptr)
	{
		if (entry->components == 0) // variable/unknown size - count the commas
		{
			std::string s(str + bytes_consumed);
			entry->components = std::count(s.begin(), s.end(), ',') + 1;
		}
		entry->size = entry->components * item_size;
		if (entry->data)
			free(entry->data);
		entry->data = (unsigned char *)malloc(entry->size);
	}
	size_t len = strlen(str);
	for (unsigned i = 0; i < entry->components; i++)
	{
		if (bytes_consumed >= len)
			throw std::runtime_error("too few parameters for EXIF tag " + tag_string);
		unsigned char *dest = entry->data + i * item_size;
		int extra_consumed = (exif_read_functions[entry->format])(str + bytes_consumed, dest);
		bytes_consumed += extra_consumed + 1; // allow a comma
	}
}

static void YUYV_to_JPEG(const uint8_t *input, const int input_width, const int input_height, const int stride,
						 const int output_width, const int output_height, const int quality, const unsigned int restart,
						 uint8_t *&jpeg_buffer, jpeg_mem_len_t &jpeg_len)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	cinfo.image_width = output_width;
	cinfo.image_height = output_height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = restart;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, quality, TRUE);
	jpeg_buffer = NULL;
	jpeg_len = 0;
	jpeg_mem_dest(&cinfo, &jpeg_buffer, &jpeg_len);
	jpeg_start_compress(&cinfo, TRUE);

	const unsigned int output_width3 = 3 * output_width;
	std::vector<uint8_t> tmp_row(output_width3);
	JSAMPROW jrow[1];
	jrow[0] = &tmp_row[0];

	// Pre-calculate the horizontal offsets to speed up the main loop.
	std::vector<unsigned int> h_offset(output_width3);
	for (unsigned int i = 0, k = 0; i < output_width; i++)
	{
		unsigned int off = (i * input_width) / output_width * 2;
		unsigned int off_align = off & ~3;
		h_offset[k++] = off;
		h_offset[k++] = off_align + 1;
		h_offset[k++] = off_align + 3;
	}
	while (cinfo.next_scanline < output_height)
	{
		unsigned int offset = ((cinfo.next_scanline * input_height) / output_height) * stride;
		for (unsigned int k = 0; k < output_width3; k += 3)
		{
			tmp_row[k] = input[offset + h_offset[k]];
			tmp_row[k + 1] = input[offset + h_offset[k + 1]];
			tmp_row[k + 2] = input[offset + h_offset[k + 2]];
		}
		jpeg_write_scanlines(&cinfo, jrow, 1);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
}

static void YUV420_to_JPEG_fast(const uint8_t *input, const int width, const int height, const int stride,
								const int quality, const unsigned int restart, uint8_t *&jpeg_buffer,
								jpeg_mem_len_t &jpeg_len)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	cinfo.image_width = width;
	cinfo.image_height = height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = restart;

	jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = TRUE;
	jpeg_set_quality(&cinfo, quality, TRUE);
	jpeg_buffer = NULL;
	jpeg_len = 0;
	jpeg_mem_dest(&cinfo, &jpeg_buffer, &jpeg_len);
	jpeg_start_compress(&cinfo, TRUE);

	int stride2 = stride / 2;
	uint8_t *Y = (uint8_t *)input;
	uint8_t *U = (uint8_t *)Y + stride * height;
	uint8_t *V = (uint8_t *)U + stride2 * (height / 2);

	JSAMPROW y_rows[16];
	JSAMPROW u_rows[8];
	JSAMPROW v_rows[8];

	int height_align = height & ~15;
	while (cinfo.next_scanline < height_align)
	{
		uint8_t *Y_row = Y + cinfo.next_scanline * stride;
		for (int i = 0; i < 16; i++, Y_row += stride)
			y_rows[i] = Y_row;
		uint8_t *U_row = U + (cinfo.next_scanline / 2) * stride2;
		uint8_t *V_row = V + (cinfo.next_scanline / 2) * stride2;
		for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
			u_rows[i] = U_row, v_rows[i] = V_row;

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}
	if (cinfo.next_scanline < height)
	{
		// Raw data has to be written in blocks of 16 rows, so rows beyond the highest
		// multiple of 16 have to be copied to a 16-row sized buffer and then added.
		std::vector<uint8_t> y_pixels(16 * stride);
		std::vector<uint8_t> u_pixels(8 * stride2);
		std::vector<uint8_t> v_pixels(8 * stride2);
		memcpy(&y_pixels[0], Y + height_align * stride, (height & 15) * stride);
		memcpy(&u_pixels[1], U + height_align / 2 * stride2, (height & 15) / 2 * stride2);
		memcpy(&v_pixels[1], V + height_align / 2 * stride2, (height & 15) / 2 * stride2);

		for (int i = 0; i < 16; i++)
			y_rows[i] = &y_pixels[i * stride];
		for (int i = 0; i < 8; i++)
			u_rows[i] = &u_pixels[i * stride2], v_rows[i] = &v_pixels[i * stride2];

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
}

static void YUV420_to_JPEG(const uint8_t *input, const int input_width, const int input_height, const int stride,
						   const int output_width, const int output_height, const int quality,
						   const unsigned int restart, uint8_t *&jpeg_buffer, jpeg_mem_len_t &jpeg_len)
{
	if (input_width == output_width && input_height == output_height)
	{
		YUV420_to_JPEG_fast(input, input_width, input_height, stride, quality, restart, jpeg_buffer, jpeg_len);
		return;
	}

	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	cinfo.image_width = output_width;
	cinfo.image_height = output_height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = restart;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, quality, TRUE);
	jpeg_buffer = NULL;
	jpeg_len = 0;
	jpeg_mem_dest(&cinfo, &jpeg_buffer, &jpeg_len);
	jpeg_start_compress(&cinfo, TRUE);

	const unsigned int output_width3 = 3 * output_width;
	std::vector<uint8_t> tmp_row(output_width3);
	JSAMPROW jrow[1];
	jrow[0] = &tmp_row[0];

	const uint8_t *Y = input;
	const uint8_t *U = Y + stride * input_height;
	const uint8_t *V = U + (stride / 2) * (input_height / 2);

	// Pre-calculate the horizontal offsets to speed up the main loop.
	std::vector<unsigned int> h_offset(output_width3);
	for (unsigned int i = 0, k = 0; i < output_width; i++)
	{
		unsigned int off = (i * input_width) / output_width;
		h_offset[k++] = off;
		h_offset[k++] = off / 2;
		h_offset[k++] = off / 2;
	}
	while (cinfo.next_scanline < output_height)
	{
		unsigned int offset = ((cinfo.next_scanline * input_height) / output_height) * stride;
		unsigned int offset_uv = (((cinfo.next_scanline / 2) * input_height) / output_height) * (stride / 2);
		for (unsigned int k = 0; k < output_width3; k += 3)
		{
			tmp_row[k] = Y[offset + h_offset[k]];
			tmp_row[k + 1] = U[offset_uv + h_offset[k + 1]];
			tmp_row[k + 2] = V[offset_uv + h_offset[k + 2]];
		}
		jpeg_write_scanlines(&cinfo, jrow, 1);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
}

static void YUV_to_JPEG(PixelFormat const &pixel_format, const uint8_t *input, const int input_width,
						const int input_height, const int stride, const int output_width, const int output_height,
						const int quality, const unsigned int restart, uint8_t *&jpeg_buffer, jpeg_mem_len_t &jpeg_len)
{
	if (pixel_format == libcamera::formats::YUYV)
		YUYV_to_JPEG(input, input_width, input_height, stride, output_width, output_height, quality, restart,
					 jpeg_buffer, jpeg_len);
	else if (pixel_format == libcamera::formats::YUV420)
		YUV420_to_JPEG(input, input_width, input_height, stride, output_width, output_height, quality, restart,
					   jpeg_buffer, jpeg_len);
	else
		throw std::runtime_error("unsupported YUV format in JPEG encode");
}

static void create_exif_data(PixelFormat const &pixel_format, std::vector<libcamera::Span<uint8_t>> const &mem, int w,
							 int h, int stride, ControlList const &metadata, std::string const &cam_name,
							 StillOptions const *options, uint8_t *&exif_buffer, unsigned int &exif_len,
							 uint8_t *&thumb_buffer, jpeg_mem_len_t &thumb_len)
{
	exif_buffer = nullptr;
	ExifData *exif = nullptr;

	try
	{
		exif = exif_data_new();
		if (!exif)
			throw std::runtime_error("failed to allocate EXIF data");
		exif_data_set_byte_order(exif, exif_byte_order);

		// First add some fixed EXIF tags.

		ExifEntry *entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_MAKE);
		exif_set_string(entry, "Raspberry Pi");
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_MODEL);
		exif_set_string(entry, cam_name.c_str());
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_SOFTWARE);
		exif_set_string(entry, "libcamera-still");
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME);
		std::time_t raw_time;
		std::time(&raw_time);
		std::tm *time_info;
		char time_string[32];
		time_info = std::localtime(&raw_time);
		std::strftime(time_string, sizeof(time_string), "%Y:%m:%d %H:%M:%S", time_info);
		exif_set_string(entry, time_string);

		// Now add some tags filled in from the image metadata.
		if (metadata.contains(libcamera::controls::ExposureTime))
		{
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_EXPOSURE_TIME);
			int32_t exposure_time = metadata.get(libcamera::controls::ExposureTime);
			if (options->verbose)
				std::cout << "Exposure time: " << exposure_time << std::endl;
			ExifRational exposure = { (ExifLong)exposure_time, 1000000 };
			exif_set_rational(entry->data, exif_byte_order, exposure);
		}
		if (metadata.contains(libcamera::controls::AnalogueGain))
		{
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_ISO_SPEED_RATINGS);
			float ag = metadata.get(libcamera::controls::AnalogueGain), dg = 1.0, gain;
			if (metadata.contains(libcamera::controls::DigitalGain))
				dg = metadata.get(libcamera::controls::DigitalGain);
			gain = ag * dg;
			if (options->verbose)
				std::cout << "Ag " << ag << " Dg " << dg << " Total " << gain << std::endl;
			exif_set_short(entry->data, exif_byte_order, 100 * gain);
		}

		// Command-line supplied tags.
		for (auto &exif_item : options->exif)
		{
			if (options->verbose)
				std::cout << "Processing EXIF item: " << exif_item << std::endl;
			exif_read_tag(exif, exif_item.c_str());
		}

		// Add some tags for the thumbnail. We put in dummy values for the thumbnail
		// offset/length to occupy the right amount of space, and fill them in later.

		if (options->verbose)
			std::cout << "Thumbnail dimensions are " << options->thumb_width << " x " << options->thumb_height
					  << std::endl;
		entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_IMAGE_WIDTH);
		exif_set_short(entry->data, exif_byte_order, options->thumb_width);
		entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_IMAGE_LENGTH);
		exif_set_short(entry->data, exif_byte_order, options->thumb_height);
		entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_COMPRESSION);
		exif_set_short(entry->data, exif_byte_order, 6);
		ExifEntry *thumb_offset_entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_JPEG_INTERCHANGE_FORMAT);
		exif_set_long(thumb_offset_entry->data, exif_byte_order, 0);
		ExifEntry *thumb_length_entry = exif_create_tag(exif, EXIF_IFD_1, EXIF_TAG_JPEG_INTERCHANGE_FORMAT_LENGTH);
		exif_set_long(thumb_length_entry->data, exif_byte_order, 0);

		// We actually have to write out an EXIF buffer to find out how long it is.

		exif_len = 0;
		exif_data_save_data(exif, &exif_buffer, &exif_len);
		free(exif_buffer);
		exif_buffer = nullptr;

		// Next create the JPEG for the thumbnail, we need to do this now so that we can
		// go back and fill in the correct values for the thumbnail offsets/length.

		int q = options->thumb_quality;
		for (; q > 0; q -= 5)
		{
			YUV_to_JPEG(pixel_format, (uint8_t *)(mem[0].data()), w, h, stride, options->thumb_width,
						options->thumb_height, q, 0, thumb_buffer, thumb_len);
			if (thumb_len < 60000) // entire EXIF data must be < 65536, so this should be safe
				break;
			free(thumb_buffer);
			thumb_buffer = nullptr;
		}
		if (options->verbose)
			std::cout << "Thumbnail size " << thumb_len << std::endl;
		if (q <= 0)
			throw std::runtime_error("failed to make acceptable thumbnail");

		// Now fill in the correct offsets and length.

		unsigned int offset = exif_len - 6; // do not ask me why "- 6", I have no idea
		exif_set_long(thumb_offset_entry->data, exif_byte_order, offset);
		exif_set_long(thumb_length_entry->data, exif_byte_order, thumb_len);

		// And create the EXIF data buffer *again*.

		exif_data_save_data(exif, &exif_buffer, &exif_len);
		exif_data_unref(exif);
		exif = nullptr;
	}
	catch (std::exception const &e)
	{
		if (exif)
			exif_data_unref(exif);
		if (exif_buffer)
			free(exif_buffer);
		if (thumb_buffer)
			free(thumb_buffer);
		throw;
	}
}

void jpeg_save(std::vector<libcamera::Span<uint8_t>> const &mem, int w, int h, int stride,
			   PixelFormat const &pixel_format, ControlList const &metadata, std::string const &filename,
			   std::string const &cam_name, StillOptions const *options)
{
	FILE *fp = nullptr;
	uint8_t *thumb_buffer = nullptr;
	unsigned char *exif_buffer = nullptr;
	uint8_t *jpeg_buffer = nullptr;

	try
	{
		if ((w & 1) || (h & 1))
			throw std::runtime_error("both width and height must be even");
		if (mem.size() != 1)
			throw std::runtime_error("only single plane YUV supported");

		// Make all the EXIF data, which includes the thumbnail.

		jpeg_mem_len_t thumb_len;
		unsigned int exif_len;
		create_exif_data(pixel_format, mem, w, h, stride, metadata, cam_name, options, exif_buffer, exif_len,
						 thumb_buffer, thumb_len);

		// Make the full size JPEG (could probably be more efficient if we had
		// YUV422 or YUV420 planar format).

		jpeg_mem_len_t jpeg_len;
		YUV_to_JPEG(pixel_format, (uint8_t *)(mem[0].data()), w, h, stride, w, h, options->quality, options->restart,
					jpeg_buffer, jpeg_len);
		if (options->verbose)
			std::cout << "JPEG size is " << jpeg_len << std::endl;

		// Write everything out.

		fp = fopen(filename.c_str(), "w");
		if (!fp)
			throw std::runtime_error("failed to open file " + options->output);

		if (options->verbose)
			std::cout << "EXIF data len " << exif_len << std::endl;

		if (fwrite(exif_header, sizeof(exif_header), 1, fp) != 1 || fputc((exif_len + thumb_len + 2) >> 8, fp) == EOF ||
			fputc((exif_len + thumb_len + 2) & 0xff, fp) == EOF || fwrite(exif_buffer, exif_len, 1, fp) != 1 ||
			fwrite(thumb_buffer, thumb_len, 1, fp) != 1 ||
			fwrite(jpeg_buffer + exif_image_offset, jpeg_len - exif_image_offset, 1, fp) != 1)
			throw std::runtime_error("failed to write file - output probably corrupt");

		fclose(fp);
		fp = nullptr;

		free(exif_buffer);
		exif_buffer = nullptr;
		free(thumb_buffer);
		thumb_buffer = nullptr;
		free(jpeg_buffer);
		jpeg_buffer = nullptr;
	}
	catch (std::exception const &e)
	{
		if (fp)
			fclose(fp);
		free(exif_buffer);
		free(thumb_buffer);
		free(jpeg_buffer);
		throw;
	}
}
