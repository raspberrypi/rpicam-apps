/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * dng.cpp - Save raw image as DNG file.
 */

#include <map>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <tiffio.h>

#include "core/still_options.hpp"
#include "core/stream_info.hpp"

using namespace libcamera;

static char TIFF_RGGB[4] = { 0, 1, 1, 2 };
static char TIFF_GRBG[4] = { 1, 0, 2, 1 };
static char TIFF_BGGR[4] = { 2, 1, 1, 0 };
static char TIFF_GBRG[4] = { 1, 2, 0, 1 };

struct BayerFormat
{
	char const *name;
	int bits;
	char const *order;
};

static const std::map<PixelFormat, BayerFormat> bayer_formats =
{
	{ formats::SRGGB10_CSI2P, { "RGGB-10", 10, TIFF_RGGB } },
	{ formats::SGRBG10_CSI2P, { "GRBG-10", 10, TIFF_GRBG } },
	{ formats::SBGGR10_CSI2P, { "BGGR-10", 10, TIFF_BGGR } },
	{ formats::SGBRG10_CSI2P, { "GBRG-10", 10, TIFF_GBRG } },
	{ formats::SRGGB12_CSI2P, { "RGGB-12", 12, TIFF_RGGB } },
	{ formats::SGRBG12_CSI2P, { "GRBG-12", 12, TIFF_GRBG } },
	{ formats::SBGGR12_CSI2P, { "BGGR-12", 12, TIFF_BGGR } },
	{ formats::SGBRG12_CSI2P, { "GBRG-12", 12, TIFF_GBRG } },
};

static void unpack_10bit(uint8_t *src, StreamInfo const &info, uint16_t *dest)
{
	unsigned int w_align = info.width & ~3;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t *ptr = src;
		unsigned int x;
		for (x = 0; x < w_align; x += 4, ptr += 5)
		{
			*dest++ = (ptr[0] << 2) | ((ptr[4] >> 0) & 3);
			*dest++ = (ptr[1] << 2) | ((ptr[4] >> 2) & 3);
			*dest++ = (ptr[2] << 2) | ((ptr[4] >> 4) & 3);
			*dest++ = (ptr[3] << 2) | ((ptr[4] >> 6) & 3);
		}
		for (; x < info.width; x++)
			*dest++ = (ptr[x & 3] << 2) | ((ptr[4] >> ((x & 3) << 1)) & 3);
	}
}

static void unpack_12bit(uint8_t *src, StreamInfo const &info, uint16_t *dest)
{
	unsigned int w_align = info.width & ~1;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t *ptr = src;
		unsigned int x;
		for (x = 0; x < w_align; x += 2, ptr += 3)
		{
			*dest++ = (ptr[0] << 4) | ((ptr[2] >> 0) & 15);
			*dest++ = (ptr[1] << 4) | ((ptr[2] >> 4) & 15);
		}
		if (x < info.width)
			*dest++ = (ptr[x & 1] << 4) | ((ptr[2] >> ((x & 1) << 2)) & 15);
	}
}

struct Matrix
{
Matrix(float m0, float m1, float m2,
	   float m3, float m4, float m5,
	   float m6, float m7, float m8)
	{
		m[0] = m0, m[1] = m1, m[2] = m2;
		m[3] = m3, m[4] = m4, m[5] = m5;
		m[6] = m6, m[7] = m7, m[8] = m8;
	}
	Matrix(float diag0, float diag1, float diag2) : Matrix(diag0, 0, 0, 0, diag1, 0, 0, 0, diag2) {}
	Matrix() {}
	float m[9];
	Matrix T() const
	{
		return Matrix(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
	}
	Matrix C() const
	{
		return Matrix(m[4] * m[8] - m[5] * m[7], -(m[3] * m[8] - m[5] * m[6]), m[3] * m[7] - m[4] * m[6],
					  -(m[1] * m[8] - m[2] * m[7]), m[0] * m[8] - m[2] * m[6], -(m[0] * m[7] - m[1] * m[6]),
					  m[1] * m[5] - m[2] * m[4], -(m[0] * m[5] - m[2] * m[3]), m[0] * m[4] - m[1] * m[3]);
	}
	Matrix Adj() const { return C().T(); }
	float Det() const
	{
		return (m[0] * (m[4] * m[8] - m[5] * m[7]) -
				m[1] * (m[3] * m[8] - m[5] * m[6]) +
				m[2] * (m[3] * m[7] - m[4] * m[6]));
	}
	Matrix Inv() const { return Adj() * (1.0 / Det()); }
	Matrix operator*(Matrix const &other) const
	{
		Matrix result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.m[i * 3 + j] =
					m[i * 3] * other.m[j] + m[i * 3 + 1] * other.m[3 + j] + m[i * 3 + 2] * other.m[6 + j];
		return result;
	}
	Matrix operator*(float const &f) const
	{
		Matrix result;
		for (int i = 0; i < 9; i++)
			result.m[i] = m[i] * f;
		return result;
	}
};

void dng_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			  ControlList const &metadata, std::string const &filename,
			  std::string const &cam_name, StillOptions const *options)
{
	// Check the Bayer format and unpack it to u16.

	auto it = bayer_formats.find(info.pixel_format);
	if (it == bayer_formats.end())
		throw std::runtime_error("unsupported Bayer format");
	BayerFormat const &bayer_format = it->second;
	std::cerr << "Bayer format is " << bayer_format.name << "\n";

	std::vector<uint16_t> buf(info.width * info.height);
	if (bayer_format.bits == 10)
		unpack_10bit((uint8_t *)mem[0].data(), info, &buf[0]);
	else if (bayer_format.bits == 12)
		unpack_12bit((uint8_t *)mem[0].data(), info, &buf[0]);
	else
		throw std::runtime_error("unsupported bit depth " + std::to_string(bayer_format.bits));

	// We need to fish out some metadata values for the DNG.

	float black = 4096 * (1 << bayer_format.bits) / 65536.0;
	float black_levels[] = { black, black, black, black };
	if (metadata.contains(controls::SensorBlackLevels))
	{
		Span<const int32_t> levels = metadata.get(controls::SensorBlackLevels);
		// levels is in the order R, Gr, Gb, B. Re-order it for the actual bayer order.
		for (int i = 0; i < 4; i++)
		{
			int j = bayer_format.order[i];
			j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!bayer_format.order[i ^ 1]);
			black_levels[j] = levels[i] * (1 << bayer_format.bits) / 65536.0;
		}
	}
	else
		std::cerr << "WARNING: no black level found, using default" << std::endl;

	float exp_time = 10000;
	if (metadata.contains(controls::ExposureTime))
		exp_time = metadata.get(controls::ExposureTime);
	else
		std::cerr << "WARNING: default to exposure time of " << exp_time << "us" << std::endl;
	exp_time /= 1e6;

	uint16_t iso = 100;
	if (metadata.contains(controls::AnalogueGain))
		iso = metadata.get(controls::AnalogueGain) * 100.0;
	else
		std::cerr << "WARNING: default to ISO value of " << iso << std::endl;

	float NEUTRAL[] = { 1, 1, 1 };
	Matrix WB_GAINS(1, 1, 1);
	if (metadata.contains(controls::ColourGains))
	{
		Span<const float> colour_gains = metadata.get(controls::ColourGains);
		NEUTRAL[0] = 1.0 / colour_gains[0];
		NEUTRAL[2] = 1.0 / colour_gains[1];
		WB_GAINS = Matrix(colour_gains[0], 1, colour_gains[1]);
	}

	// Use a slightly plausible default CCM in case the metadata doesn't have one (it should!).
	Matrix CCM(1.90255, -0.77478, -0.12777,
			   -0.31338, 1.88197, -0.56858,
			   -0.06001, -0.61785, 1.67786);
	if (metadata.contains(controls::ColourCorrectionMatrix))
	{
		Span<const float> const &coeffs = metadata.get(controls::ColourCorrectionMatrix);
		CCM = Matrix(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8]);
	}
	else
		std::cerr << "WARNING: no CCM metadata found" << std::endl;

	// This maxtrix from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
	Matrix RGB2XYZ(0.4124564, 0.3575761, 0.1804375,
				   0.2126729, 0.7151522, 0.0721750,
				   0.0193339, 0.1191920, 0.9503041);
	Matrix CAM_XYZ = (RGB2XYZ * CCM * WB_GAINS).Inv();

	if (options->verbose)
	{
		std::cerr << "Black levels " << black_levels[0] << " " << black_levels[1] << " " << black_levels[2] << " "
				  << black_levels[3] << ", exposure time " << exp_time * 1e6 << "us, ISO " << iso << std::endl;
		std::cerr << "Neutral " << NEUTRAL[0] << " " << NEUTRAL[1] << " " << NEUTRAL[2] << std::endl;
		std::cerr << "Cam_XYZ: " << std::endl;
		std::cerr << CAM_XYZ.m[0] << " " << CAM_XYZ.m[1] << " " << CAM_XYZ.m[2] << std::endl;
		std::cerr << CAM_XYZ.m[3] << " " << CAM_XYZ.m[4] << " " << CAM_XYZ.m[5] << std::endl;
		std::cerr << CAM_XYZ.m[6] << " " << CAM_XYZ.m[7] << " " << CAM_XYZ.m[8] << std::endl;
	}

	// Finally write the DNG.

	TIFF *tif = nullptr;

	try
	{
		const short cfa_repeat_pattern_dim[] = { 2, 2 };
		uint32_t white = (1 << bayer_format.bits) - 1;
		toff_t offset_subifd = 0, offset_exififd = 0;

		tif = TIFFOpen(filename.c_str(), "w");
		if (!tif)
			throw std::runtime_error("could not open file " + filename);

		// This is just the thumbnail, but put it first to help software that only
		// reads the first IFD.
		TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 1);
		TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, info.width >> 4);
		TIFFSetField(tif, TIFFTAG_IMAGELENGTH, info.height >> 4);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_MAKE, "Raspberry Pi");
		TIFFSetField(tif, TIFFTAG_MODEL, cam_name.c_str());
		TIFFSetField(tif, TIFFTAG_DNGVERSION, "\001\001\000\000");
		TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, "\001\000\000\000");
		TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, cam_name.c_str());
		TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(tif, TIFFTAG_SOFTWARE, "libcamera-still");
		TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, CAM_XYZ.m);
		TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, NEUTRAL);
		TIFFSetField(tif, TIFFTAG_CALIBRATIONILLUMINANT1, 21);
		TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
		TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);

		// Make a small greyscale thumbnail, just to give some clue what's in here.
		std::vector<uint8_t> thumb_buf((info.width >> 4) * 3);

		for (unsigned int y = 0; y < (info.height >> 4); y++)
		{
			for (unsigned int x = 0; x < (info.width >> 4); x++)
			{
				unsigned int off = (y * info.width + x) << 4;
				int grey = buf[off] + buf[off + 1] + buf[off + info.width] + buf[off + info.width + 1];
				grey = white * sqrt(grey / (double)white); // fake "gamma"
				thumb_buf[3 * x] = thumb_buf[3 * x + 1] = thumb_buf[3 * x + 2] = grey >> (bayer_format.bits - 6);
			}
			if (TIFFWriteScanline(tif, &thumb_buf[0], y, 0) != 1)
				throw std::runtime_error("error writing DNG thumbnail data");
		}

		TIFFWriteDirectory(tif);

		// The main image (actually tends to show up as "sub-image 1").
		TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
		TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, info.width);
		TIFFSetField(tif, TIFFTAG_IMAGELENGTH, info.height);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_CFA);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, cfa_repeat_pattern_dim);
#if TIFFLIB_VERSION >= 20201219 // version 4.2.0 or later
		TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, bayer_format.order);
#else
		TIFFSetField(tif, TIFFTAG_CFAPATTERN, bayer_format.order);
#endif
		TIFFSetField(tif, TIFFTAG_WHITELEVEL, 1, &white);
		const uint16_t black_level_repeat_dim[] = { 2, 2 };
		TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, &black_level_repeat_dim);
		TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 4, &black_levels);

		for (unsigned int y = 0; y < info.height; y++)
		{
			if (TIFFWriteScanline(tif, &buf[info.width * y], y, 0) != 1)
				throw std::runtime_error("error writing DNG image data");
		}

		// We have to checkpoint before the directory offset is valid.
		TIFFCheckpointDirectory(tif);
		offset_subifd = TIFFCurrentDirOffset(tif);
		TIFFWriteDirectory(tif);

		// Create a separate IFD just for the EXIF tags. Why we couldn't simply have
		// DNG tags for these, which would have made life so much easier, I have no idea.
		TIFFCreateEXIFDirectory(tif);

		time_t t;
		time(&t);
		struct tm *time_info = localtime(&t);
		char time_str[32];
		strftime(time_str, 32, "%Y:%m:%d %H:%M:%S", time_info);
		TIFFSetField(tif, EXIFTAG_DATETIMEORIGINAL, time_str);

		TIFFSetField(tif, EXIFTAG_ISOSPEEDRATINGS, 1, &iso);
		TIFFSetField(tif, EXIFTAG_EXPOSURETIME, exp_time);

		TIFFCheckpointDirectory(tif);
		offset_exififd = TIFFCurrentDirOffset(tif);
		TIFFWriteDirectory(tif);

		// Now got back to the initial IFD and correct the offsets to its sub-thingies
		TIFFSetDirectory(tif, 0);
		TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
		TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);
		TIFFWriteDirectory(tif);

		// For reasons unknown, the last sub-IFD that we make seems to reappear at the
		// end of the file as IDF1, and some tools (exiftool for example) are prone to
		// complain about it. As far as I can see the code above is doing the correct
		// things, and I can't find any references to this problem anywhere. So frankly
		// I have no idea what is happening - please let us know if you do. Anyway,
		// this bodge appears to make the problem go away...
		TIFFUnlinkDirectory(tif, 2);

		TIFFClose(tif);
	}
	catch (std::exception const &e)
	{
		if (tif)
			TIFFClose(tif);
		throw;
	}
}
