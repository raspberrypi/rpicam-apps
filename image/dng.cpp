/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * dng.cpp - Save raw image as DNG file.
 */

#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <map>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <tiffio.h>

#include "core/still_options.hpp"
#include "core/stream_info.hpp"

#ifndef MAKE_STRING
#define MAKE_STRING "Raspberry Pi"
#endif

using namespace libcamera;

#ifndef PHOTOMETRIC_LINEARRAW
#define PHOTOMETRIC_LINEARRAW 34892
#endif

using namespace libcamera;

static char TIFF_RGGB[4] = { 0, 1, 1, 2 };
static char TIFF_GRBG[4] = { 1, 0, 2, 1 };
static char TIFF_BGGR[4] = { 2, 1, 1, 0 };
static char TIFF_GBRG[4] = { 1, 2, 0, 1 };

struct FormatInfo
{
	char const *name;
	int bits;
	char const *order;
	bool packed;
	bool compressed;
};

// clang-format off
static const std::map<PixelFormat, FormatInfo> bayer_formats =
{
	{ formats::SRGGB10_CSI2P, { "RGGB-10", 10, TIFF_RGGB, true, false } },
	{ formats::SGRBG10_CSI2P, { "GRBG-10", 10, TIFF_GRBG, true, false } },
	{ formats::SBGGR10_CSI2P, { "BGGR-10", 10, TIFF_BGGR, true, false } },
	{ formats::SGBRG10_CSI2P, { "GBRG-10", 10, TIFF_GBRG, true, false } },

	{ formats::SRGGB10, { "RGGB-10", 10, TIFF_RGGB, false, false } },
	{ formats::SGRBG10, { "GRBG-10", 10, TIFF_GRBG, false, false } },
	{ formats::SBGGR10, { "BGGR-10", 10, TIFF_BGGR, false, false } },
	{ formats::SGBRG10, { "GBRG-10", 10, TIFF_GBRG, false, false } },

	{ formats::SRGGB12_CSI2P, { "RGGB-12", 12, TIFF_RGGB, true, false } },
	{ formats::SGRBG12_CSI2P, { "GRBG-12", 12, TIFF_GRBG, true, false } },
	{ formats::SBGGR12_CSI2P, { "BGGR-12", 12, TIFF_BGGR, true, false } },
	{ formats::SGBRG12_CSI2P, { "GBRG-12", 12, TIFF_GBRG, true, false } },

	{ formats::SRGGB12, { "RGGB-12", 12, TIFF_RGGB, false, false } },
	{ formats::SGRBG12, { "GRBG-12", 12, TIFF_GRBG, false, false } },
	{ formats::SBGGR12, { "BGGR-12", 12, TIFF_BGGR, false, false } },
	{ formats::SGBRG12, { "GBRG-12", 12, TIFF_GBRG, false, false } },

	{ formats::SRGGB16, { "RGGB-16", 16, TIFF_RGGB, false, false } },
	{ formats::SGRBG16, { "GRBG-16", 16, TIFF_GRBG, false, false } },
	{ formats::SBGGR16, { "BGGR-16", 16, TIFF_BGGR, false, false } },
	{ formats::SGBRG16, { "GBRG-16", 16, TIFF_GBRG, false, false } },

	/* PiSP compressed formats. */
	{ formats::RGGB_PISP_COMP1, { "RGGB-16-PISP", 16, TIFF_RGGB, false, true } },
	{ formats::GRBG_PISP_COMP1, { "GRBG-16-PISP", 16, TIFF_GRBG, false, true } },
	{ formats::GBRG_PISP_COMP1, { "GBRG-16-PISP", 16, TIFF_GBRG, false, true } },
	{ formats::BGGR_PISP_COMP1, { "BGGR-16-PISP", 16, TIFF_BGGR, false, true } },
};

static const std::map<PixelFormat, FormatInfo> mono_formats =
{
	/* Monochrome formats */
	{ formats::R10_CSI2P, { "MONO-10", 10, NULL, true, false } },
	{ formats::R12_CSI2P, { "MONO-12", 12, NULL, true, false } },
	{ formats::R16,		  { "MONO-16", 16, NULL, false, false } },

	/* Monochrome + PISP compressed format */
	{ formats::MONO_PISP_COMP1, { "MONO-16-PISP", 16, NULL, false, true } },
};
// clang-format on

static bool is_mono(const PixelFormat &format)
{
	auto it = mono_formats.find(format);
	return it != mono_formats.end();
}

static void unpack_10bit(uint8_t const *src, StreamInfo const &info, uint16_t *dest)
{
	unsigned int w_align = info.width & ~3;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t const *ptr = src;
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

static void unpack_12bit(uint8_t const *src, StreamInfo const &info, uint16_t *dest)
{
	unsigned int w_align = info.width & ~1;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t const *ptr = src;
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

static void unpack_16bit(uint8_t const *src, StreamInfo const &info, uint16_t *dest)
{
	/* Assume the pixels in memory are already in native byte order */
	unsigned int w = info.width;
	for (unsigned int y = 0; y < info.height; y++)
	{
		memcpy(dest, src, 2 * w);
		dest += w;
		src += info.stride;
	}
}

// We always use these compression parameters.
#define COMPRESS_OFFSET 2048
#define COMPRESS_MODE 1

static uint16_t postprocess(uint16_t a)
{
	if (COMPRESS_MODE & 2)
	{
		if (COMPRESS_MODE == 3 && a < 0x4000)
			a = a >> 2;
		else if (a < 0x1000)
			a = a >> 4;
		else if (a < 0x1800)
			a = (a - 0x800) >> 3;
		else if (a < 0x3000)
			a = (a - 0x1000) >> 2;
		else if (a < 0x6000)
			a = (a - 0x2000) >> 1;
		else if (a < 0xC000)
			a = (a - 0x4000);
		else
			a = 2 * (a - 0x8000);
	}

	return std::min(0xFFFF, a + COMPRESS_OFFSET);
}

static uint16_t dequantize(uint16_t q, int qmode)
{
	switch (qmode)
	{
	case 0:
		return (q < 320) ? 16 * q : 32 * (q - 160);

	case 1:
		return 64 * q;

	case 2:
		return 128 * q;

	default:
		return (q < 94) ? 256 * q : std::min(0xFFFF, 512 * (q - 47));
	}
}

static void subBlockFunction(uint16_t *d, uint32_t w)
{
	int q[4];

	int qmode = (w & 3);
	if (qmode < 3)
	{
		int field0 = (w >> 2) & 511;
		int field1 = (w >> 11) & 127;
		int field2 = (w >> 18) & 127;
		int field3 = (w >> 25) & 127;
		if (qmode == 2 && field0 >= 384)
		{
			q[1] = field0;
			q[2] = field1 + 384;
		}
		else
		{
			q[1] = (field1 >= 64) ? field0 : field0 + 64 - field1;
			q[2] = (field1 >= 64) ? field0 + field1 - 64 : field0;
		}
		int p1 = std::max(0, q[1] - 64);
		if (qmode == 2)
			p1 = std::min(384, p1);
		int p2 = std::max(0, q[2] - 64);
		if (qmode == 2)
			p2 = std::min(384, p2);
		q[0] = p1 + field2;
		q[3] = p2 + field3;
	}
	else
	{
		int pack0 = (w >> 2) & 32767;
		int pack1 = (w >> 17) & 32767;
		q[0] = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
		q[1] = (pack0 >> 4) % 176;
		q[2] = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
		q[3] = (pack1 >> 4) % 176;
	}

	d[0] = dequantize(q[0], qmode);
	d[2] = dequantize(q[1], qmode);
	d[4] = dequantize(q[2], qmode);
	d[6] = dequantize(q[3], qmode);
}

static void uncompress(uint8_t const *src, StreamInfo const &info, uint16_t *dest)
{
	// In all cases, the *decompressed* image must be a multiple of 8 columns wide.
	unsigned int buf_stride_pixels = (info.width + 7) & ~7;
	for (unsigned int y = 0; y < info.height; ++y)
	{
		uint16_t *dp = dest + y * buf_stride_pixels;
		uint8_t const *sp = src + y * info.stride;

		for (unsigned int x = 0; x < info.width; x += 8)
		{
			if (COMPRESS_MODE & 1)
			{
				uint32_t w0 = 0, w1 = 0;
				for (int b = 0; b < 4; ++b)
					w0 |= (*sp++) << (b * 8);
				for (int b = 0; b < 4; ++b)
					w1 |= (*sp++) << (b * 8);
				subBlockFunction(dp, w0);
				subBlockFunction(dp + 1, w1);
				for (int i = 0; i < 8; ++i, ++dp)
					*dp = postprocess(*dp);
			}
			else
			{
				for (int i = 0; i < 8; ++i)
					*dp++ = postprocess((*sp++) << 8);
			}
		}
	}
}

// clang-format off
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
// clang-format on

class DNGOpcode
{
public:
	DNGOpcode() = default;
	virtual ~DNGOpcode() = default;

	DNGOpcode(const DNGOpcode &) noexcept = default;
	DNGOpcode(DNGOpcode &&) noexcept = default;
	DNGOpcode &operator=(const DNGOpcode &) noexcept = default;
	DNGOpcode &operator=(DNGOpcode &&) noexcept = default;

	virtual void SwitchEndianess() = 0;

	virtual uint32_t GetID() const = 0;

	virtual uint32_t GetDNGVersion() const
	{
		if constexpr (std::endian::native == std::endian::big)
		{
			// version 1.3 big endian
			return (1 << 24) | (3 << 16) | (0 << 8) | (0 << 0);
		}

		// version 1.3
		return (1 << 0) | (3 << 8) | (0 << 16) | (0 << 24);
	};

	uint32_t GetFlags() const
	{
		return 0;
	}

	virtual uint32_t GetSizeInBytes() const = 0;

	virtual std::vector<uint8_t> GetBytes() const = 0;

protected:
};

// GCC emits an error=unitialized, but only for float and double, no idea why...
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
template <typename T>
static void EndianSwap(T &aData)
{
	static_assert((sizeof(T) == 1U) || (sizeof(T) == 2U) || (sizeof(T) == 4U) || (sizeof(T) == 8U));

	switch (sizeof(T))
	{
	case 2:
	{
		uint16_t temp = *(reinterpret_cast<uint16_t *>(&aData));
		temp = (temp >> 8U) | (temp << 8U);
		T *tempT = reinterpret_cast<T *>(&temp);
		aData = *tempT;
		break;
	}
	case 4:
	{
		uint32_t temp = *(reinterpret_cast<uint32_t *>(&aData));
		temp = (temp >> 24U) | ((temp << 8U) & 0x00FF0000U) | ((temp >> 8U) & 0x0000FF00U) | (temp << 24U);
		T *tempT = reinterpret_cast<T *>(&temp);
		aData = *tempT;
		break;
	}
	case 8:
	{
		uint64_t temp = *(reinterpret_cast<uint64_t *>(&aData));
		temp = (temp >> 56U) | ((temp << 40U) & 0x00FF000000000000U) | ((temp << 24U) & 0x0000FF0000000000U) |
			   ((temp << 8U) & 0x000000FF00000000U) | ((temp >> 8U) & 0x00000000FF000000U) |
			   ((temp >> 24U) & 0x0000000000FF0000U) | ((temp >> 40U) & 0x000000000000FF00U) | (temp << 56U);
		T *tempT = reinterpret_cast<T *>(&temp);
		aData = *tempT;
		break;
	}
	default:
		break;
	}
}
#pragma GCC diagnostic pop

template <class Iterator>
static void EndianSwap(Iterator aFirst, Iterator aLast)
{
	std::for_each(aFirst, aLast, [](auto &aElem) { EndianSwap(aElem); });
}

class DNGOpcodeList
{
public:
	DNGOpcodeList() = default;
	~DNGOpcodeList() = default;

	DNGOpcodeList(const DNGOpcodeList &) noexcept = default;
	DNGOpcodeList(DNGOpcodeList &&) noexcept = default;
	DNGOpcodeList &operator=(const DNGOpcodeList &) noexcept = default;
	DNGOpcodeList &operator=(DNGOpcodeList &&) noexcept = default;

	void SwitchEndianess();

	uint32_t GetSizeInBytes() const;

	std::vector<uint8_t> GetBytes() const;

	void AddOpcode(DNGOpcode *aOpcode);

private:
	std::vector<DNGOpcode *> mOpcodes;
};

void DNGOpcodeList::SwitchEndianess()
{
	if constexpr (std::endian::native == std::endian::big)
	{
		return;
	}

	for (auto &elem : mOpcodes)
	{
		elem->SwitchEndianess();
	}
}

uint32_t DNGOpcodeList::GetSizeInBytes() const
{
	//                                    each opcode has a 4 uint32_t header     + number of opcodes
	uint32_t size = static_cast<uint32_t>(mOpcodes.size() * 4 * sizeof(uint32_t)) + sizeof(uint32_t);

	for (const auto &elem : mOpcodes)
	{
		size += elem->GetSizeInBytes();
	}

	return size;
}

std::vector<uint8_t> DNGOpcodeList::GetBytes() const
{
	std::array<uint8_t, 4> temp;
	std::vector<uint8_t> ret;
	ret.reserve(GetSizeInBytes());

	uint32_t opCodeCount = static_cast<uint32_t>(mOpcodes.size());
	if constexpr (std::endian::native == std::endian::little)
	{
		EndianSwap(opCodeCount);
	}
	memcpy(temp.data(), &opCodeCount, sizeof(opCodeCount));
	ret.insert(ret.end(), temp.begin(), temp.end());

	for (const DNGOpcode *op : mOpcodes)
	{
		uint32_t opCodeID = op->GetID();
		memcpy(temp.data(), &opCodeID, sizeof(opCodeID));
		ret.insert(ret.end(), temp.begin(), temp.end());

		uint32_t opDNGVersion = op->GetDNGVersion();
		memcpy(temp.data(), &opDNGVersion, sizeof(opDNGVersion));
		ret.insert(ret.end(), temp.begin(), temp.end());

		uint32_t opDNGFlags = op->GetFlags();
		memcpy(temp.data(), &opDNGFlags, sizeof(opDNGFlags));
		ret.insert(ret.end(), temp.begin(), temp.end());

		uint32_t opDNGBytes = op->GetSizeInBytes();
		if constexpr (std::endian::native == std::endian::little)
		{
			EndianSwap(opDNGBytes);
		}
		memcpy(temp.data(), &opDNGBytes, sizeof(opDNGBytes));
		ret.insert(ret.end(), temp.begin(), temp.end());

		std::vector<uint8_t> bytes = op->GetBytes();
		ret.insert(ret.end(), bytes.begin(), bytes.end());
	}

	return ret;
}

void DNGOpcodeList::AddOpcode(DNGOpcode *aOpcode)
{
	mOpcodes.emplace_back(aOpcode);
}

class DNGGainMap : public DNGOpcode
{
public:
	DNGGainMap(int aTop, int aLeft, int aBottom, int aRight, int aPlane, int aPlanes, int aRowPitch, int aColPitch,
			   int aMapPointsV, int aMapPointsH, double aMapSpacingV, double aMapSpacingH, double aMapOriginV,
			   double aMapOriginH, int aMapPlanes, std::vector<float> aValues);
	virtual ~DNGGainMap() = default;

	DNGGainMap(const DNGGainMap &) noexcept = default;
	DNGGainMap(DNGGainMap &&) noexcept = default;
	DNGGainMap &operator=(const DNGGainMap &) noexcept = default;
	DNGGainMap &operator=(DNGGainMap &&) noexcept = default;

	void SwitchEndianess() override;

	uint32_t GetID() const override;

	uint32_t GetSizeInBytes() const override;

	std::vector<uint8_t> GetBytes() const override;

private:
	int mTop;
	int mLeft;
	int mBottom;
	int mRight;
	int mPlane;
	int mPlanes;
	int mRowPitch;
	int mColPitch;
	int mMapPointsV;
	int mMapPointsH;
	double mMapSpacingV;
	double mMapSpacingH;
	double mMapOriginV;
	double mMapOriginH;
	int mMapPlanes;
	std::vector<float> mValues;
};

DNGGainMap::DNGGainMap(int aTop, int aLeft, int aBottom, int aRight, int aPlane, int aPlanes, int aRowPitch,
					   int aColPitch, int aMapPointsV, int aMapPointsH, double aMapSpacingV, double aMapSpacingH,
					   double aMapOriginV, double aMapOriginH, int aMapPlanes, std::vector<float> aValues)
	: mTop(aTop), mLeft(aLeft), mBottom(aBottom), mRight(aRight), mPlane(aPlane), mPlanes(aPlanes),
	  mRowPitch(aRowPitch), mColPitch(aColPitch), mMapPointsV(aMapPointsV), mMapPointsH(aMapPointsH),
	  mMapSpacingV(aMapSpacingV), mMapSpacingH(aMapSpacingH), mMapOriginV(aMapOriginV), mMapOriginH(aMapOriginH),
	  mMapPlanes(aMapPlanes), mValues(std::move(aValues))
{
}

void DNGGainMap::SwitchEndianess()
{
	if constexpr (std::endian::native == std::endian::big)
	{
		return;
	}

	EndianSwap(mTop);
	EndianSwap(mLeft);
	EndianSwap(mBottom);
	EndianSwap(mRight);

	EndianSwap(mPlane);
	EndianSwap(mPlanes);

	EndianSwap(mRowPitch);
	EndianSwap(mColPitch);

	EndianSwap(mMapPointsV);
	EndianSwap(mMapPointsH);

	EndianSwap(mMapSpacingV);
	EndianSwap(mMapSpacingH);
	EndianSwap(mMapOriginV);
	EndianSwap(mMapOriginH);

	EndianSwap(mMapPlanes);

	for (float &val : mValues)
	{
		EndianSwap(val);
	}
}

uint32_t DNGGainMap::GetID() const
{
	uint32_t ID = 9;
	if constexpr (std::endian::native == std::endian::little)
	{
		EndianSwap(ID);
	}
	return ID;
}

uint32_t DNGGainMap::GetSizeInBytes() const
{
	//                           all int values   + all double values  + the table(s) as floats
	return static_cast<uint32_t>(11 * sizeof(int) + 4 * sizeof(double) + mValues.size() * sizeof(float));
}

std::vector<uint8_t> DNGGainMap::GetBytes() const
{
	std::vector<uint8_t> ret(GetSizeInBytes(), 0);

	size_t offset = 0;

	memcpy(&ret[offset], &mTop, sizeof(mTop));
	offset += sizeof(mTop);
	memcpy(&ret[offset], &mLeft, sizeof(mLeft));
	offset += sizeof(mLeft);
	memcpy(&ret[offset], &mBottom, sizeof(mBottom));
	offset += sizeof(mBottom);
	memcpy(&ret[offset], &mRight, sizeof(mRight));
	offset += sizeof(mRight);

	memcpy(&ret[offset], &mPlane, sizeof(mPlane));
	offset += sizeof(mPlane);
	memcpy(&ret[offset], &mPlanes, sizeof(mPlanes));
	offset += sizeof(mPlanes);

	memcpy(&ret[offset], &mRowPitch, sizeof(mRowPitch));
	offset += sizeof(mRowPitch);
	memcpy(&ret[offset], &mColPitch, sizeof(mColPitch));
	offset += sizeof(mColPitch);

	memcpy(&ret[offset], &mMapPointsV, sizeof(mMapPointsV));
	offset += sizeof(mMapPointsV);
	memcpy(&ret[offset], &mMapPointsV, sizeof(mMapPointsV));
	offset += sizeof(mMapPointsV);

	memcpy(&ret[offset], &mMapSpacingV, sizeof(mMapSpacingV));
	offset += sizeof(mMapSpacingV);
	memcpy(&ret[offset], &mMapSpacingH, sizeof(mMapSpacingH));
	offset += sizeof(mMapSpacingH);
	memcpy(&ret[offset], &mMapOriginV, sizeof(mMapOriginV));
	offset += sizeof(mMapOriginV);
	memcpy(&ret[offset], &mMapOriginH, sizeof(mMapOriginH));
	offset += sizeof(mMapOriginH);

	memcpy(&ret[offset], &mMapPlanes, sizeof(mMapPlanes));
	offset += sizeof(mMapPlanes);

	memcpy(&ret[offset], mValues.data(), sizeof(float) * mValues.size());
	offset += sizeof(float) * mValues.size();

	return ret;
}

// By default, libtiff knows these DNG specific tags, but they are not registered for reading or writing
// --> We have to register them before using them (see tif_dirinfo.c of libtiff)
static const TIFFFieldInfo DNGExtra[] = {

	{ TIFFTAG_PROFILETONECURVE, -1, -1, TIFF_FLOAT, FIELD_CUSTOM, 1, 1, const_cast<char *>("ProfileToneCurve") },
	{ TIFFTAG_OPCODELIST1, -3, -3, TIFF_UNDEFINED, FIELD_CUSTOM, (unsigned char)1, (unsigned char)1,
	  const_cast<char *>("OpcodeList1") },
	{ TIFFTAG_OPCODELIST2, -3, -3, TIFF_UNDEFINED, FIELD_CUSTOM, (unsigned char)1, (unsigned char)1,
	  const_cast<char *>("OpcodeList2") },
	{ TIFFTAG_OPCODELIST3, -3, -3, TIFF_UNDEFINED, FIELD_CUSTOM, (unsigned char)1, (unsigned char)1,
	  const_cast<char *>("OpcodeList3") },
	{ TIFFTAG_NOISEPROFILE, -1, -1, TIFF_DOUBLE, FIELD_CUSTOM, (unsigned char)1, (unsigned char)1,
	  const_cast<char *>("NoiseProfile") },

};

static TIFFExtendProc ParentExtender = NULL;

static void RegisterTiffTagsCallback(TIFF *tif)
{
	TIFFMergeFieldInfo(tif, DNGExtra, 5);

	if (ParentExtender)
	{
		(*ParentExtender)(tif);
	}
}

static void RegisterCustomTiffTags(void)
{
	static int first_time = 1; //or better thread_local?

	if (!first_time)
		return;
	first_time = 0;

	ParentExtender = TIFFSetTagExtender(RegisterTiffTagsCallback);
}

void dng_save(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info, ControlList const &metadata,
			  std::string const &filename, std::string const &cam_model, StillOptions const *options)
{
	RegisterCustomTiffTags();

	FormatInfo format;
	bool mono = is_mono(info.pixel_format);

	if (mono)
	{
		format = mono_formats.find(info.pixel_format)->second;
		LOG(1, "Mono format is " << format.name);
	}
	else
	{
		// Check the Bayer format and unpack it to u16.
		auto it = bayer_formats.find(info.pixel_format);
		if (it == bayer_formats.end())
			throw std::runtime_error("unsupported Bayer format");
		format = it->second;
		LOG(1, "Bayer format is " << format.name);
	}

	// Decompression will require a buffer that's 8 pixels aligned.
	unsigned int buf_stride_pixels = info.width;
	unsigned int buf_stride_pixels_padded = (buf_stride_pixels + 7) & ~7;
	std::vector<uint16_t> buf(buf_stride_pixels_padded * info.height);
	if (format.compressed)
	{
		uncompress(mem[0].data(), info, &buf[0]);
		buf_stride_pixels = buf_stride_pixels_padded;
	}
	else if (format.packed)
	{
		switch (format.bits)
		{
		case 10:
			unpack_10bit(mem[0].data(), info, &buf[0]);
			break;
		case 12:
			unpack_12bit(mem[0].data(), info, &buf[0]);
			break;
		}
	}
	else
		unpack_16bit(mem[0].data(), info, &buf[0]);

	// We need to fish out some metadata values for the DNG.
	float black = 4096 * (1 << format.bits) / 65536.0;
	float black_levels[] = { black, black, black, black };
	auto bl = metadata.get(controls::SensorBlackLevels);
	if (bl)
	{
		if (!mono)
		{
			// levels is in the order R, Gr, Gb, B. Re-order it for the actual bayer order.
			for (int i = 0; i < 4; i++)
			{
				int j = format.order[i];
				j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!format.order[i ^ 1]);
				black_levels[j] = (*bl)[i] * (1 << format.bits) / 65536.0;
			}
		}
		else
			black_levels[0] = (*bl)[0] * (1 << format.bits) / 65536.0;
	}
	else
		LOG_ERROR("WARNING: no black level found, using default");

	auto exp = metadata.get(controls::ExposureTime);
	float exp_time = 10000;
	if (exp)
		exp_time = *exp;
	else
		LOG_ERROR("WARNING: default to exposure time of " << exp_time << "us");
	exp_time /= 1e6;

	auto ag = metadata.get(controls::AnalogueGain);
	uint16_t iso = 100;
	if (ag)
		iso = *ag * 100.0;
	else
		LOG_ERROR("WARNING: default to ISO value of " << iso);

	uint32_t widthLSC = 0;
	uint32_t heightLSC = 0;
	std::vector<std::vector<float>> lscMaps;

	auto lscs = metadata.get(controls::LensShadingCorrectionMapSize);
	if (lscs)
	{
		widthLSC = (*lscs)[1];
		heightLSC = (*lscs)[2];
		const uint32_t coloursCount = mono ? 1 : (*lscs)[0];
		const uint32_t tableSize = widthLSC * heightLSC;

		lscMaps.resize(coloursCount);

		lscMaps[0].resize(tableSize);
		if (coloursCount == 3)
		{
			lscMaps[1].resize(tableSize);
			lscMaps[2].resize(tableSize);
		}

		auto lscmaps = metadata.get(controls::LensShadingCorrectionMaps);
		if (lscmaps && widthLSC > 0 && heightLSC > 0)
		{
			std::copy((*lscmaps).begin(), (*lscmaps).begin() + tableSize, lscMaps[0].begin());

			if (coloursCount == 3)
			{
				std::copy((*lscmaps).begin() + 1 * tableSize, (*lscmaps).begin() + 2 * tableSize, lscMaps[1].begin());
				std::copy((*lscmaps).begin() + 2 * tableSize, (*lscmaps).begin() + 3 * tableSize, lscMaps[2].begin());
			}
		}

		if (!lscmaps || widthLSC == 0 || heightLSC == 0 || (coloursCount != 1 && coloursCount != 3))
		{
			widthLSC = 0; //reset to zero in case a map is missing
			heightLSC = 0;
			LOG_ERROR("WARNING: No lens shading correction tables found.");
		}
	}

	auto noise = metadata.get(controls::NoiseProfile);
	std::array<double, 2> noiseProfile = { 0, 0 };
	if (noise)
	{
		noiseProfile[0] = std::sqrt((*noise)[0] / 65535.0); // slope
		noiseProfile[1] = 0; // the offset value is always close to zero and I'm not sure how to convert it.
	}

	std::vector<float> toneCurve;
	auto tc = metadata.get(controls::ToneCurve);
	if (tc)
	{
		toneCurve.resize(tc->size());
		for (size_t i = 0; i < (*tc).size(); i++)
		{
			toneCurve[i] = (*tc)[i] / 65535.0f;
		}
		// The provided tone curve has a baked in sRGB gamma curve, remove it:
		for (size_t i = 1; i < (*tc).size(); i += 2)
		{
			if (toneCurve[i] < 0.04045f)
			{
				toneCurve[i] /= 12.92f;
			}
			else
			{
				toneCurve[i] = powf((toneCurve[i] + 0.055f) / 1.055f, 2.4f);
			}
		}
	}
	else
	{
		LOG_ERROR("WARNING: No tone curve table found.");
	}

	float NEUTRAL[] = { 1, 1, 1 };
	Matrix WB_GAINS(1, 1, 1);
	auto cg = metadata.get(controls::ColourGains);
	if (cg)
	{
		NEUTRAL[0] = 1.0 / (*cg)[0];
		NEUTRAL[2] = 1.0 / (*cg)[1];
		WB_GAINS = Matrix((*cg)[0], 1, (*cg)[1]);
	}

	// Use a slightly plausible default CCM in case the metadata doesn't have one (it should!).
	// clang-format off
	Matrix CCM(1.90255, -0.77478, -0.12777,
			   -0.31338, 1.88197, -0.56858,
			   -0.06001, -0.61785, 1.67786);
	// clang-format on
	auto ccm = metadata.get(controls::ColourCorrectionMatrix);
	if (ccm)
	{
		CCM = Matrix((*ccm)[0], (*ccm)[1], (*ccm)[2], (*ccm)[3], (*ccm)[4], (*ccm)[5], (*ccm)[6], (*ccm)[7], (*ccm)[8]);
	}
	else if (!mono)
		LOG_ERROR("WARNING: no CCM metadata found");

	// This maxtrix from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html (sRGB with D65 to XYZ D50)
	// clang-format off
	Matrix RGB2XYZ(0.4360747, 0.3850649, 0.1430804,
				   0.2225045, 0.7168786, 0.0606169,
				   0.0139322, 0.0971045, 0.7141733);
	// clang-format on
	Matrix CAM_XYZ = (RGB2XYZ * CCM * WB_GAINS).Inv();

	LOG(2, "Black levels " << black_levels[0] << " " << black_levels[1] << " " << black_levels[2] << " "
						   << black_levels[3] << ", exposure time " << exp_time * 1e6 << "us, ISO " << iso);
	LOG(2, "Neutral " << NEUTRAL[0] << " " << NEUTRAL[1] << " " << NEUTRAL[2]);
	LOG(2, "Cam_XYZ: ");
	LOG(2, CAM_XYZ.m[0] << " " << CAM_XYZ.m[1] << " " << CAM_XYZ.m[2]);
	LOG(2, CAM_XYZ.m[3] << " " << CAM_XYZ.m[4] << " " << CAM_XYZ.m[5]);
	LOG(2, CAM_XYZ.m[6] << " " << CAM_XYZ.m[7] << " " << CAM_XYZ.m[8]);

	// Finally write the DNG.

	TIFF *tif = nullptr;

	try
	{
		const short cfa_repeat_pattern_dim[] = { 2, 2 };
		uint32_t white = (1 << format.bits) - 1;
		toff_t offset_subifd = 0, offset_exififd = 0;
		std::string unique_model = std::string(MAKE_STRING " ") + cam_model;

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
		TIFFSetField(tif, TIFFTAG_MAKE, MAKE_STRING);
		TIFFSetField(tif, TIFFTAG_MODEL, cam_model.c_str());
		TIFFSetField(tif, TIFFTAG_DNGVERSION, "\001\003\000\000"); // version 1.3 for opcodes
		TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, "\001\000\000\000");
		TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, unique_model.c_str());
		TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(tif, TIFFTAG_SOFTWARE, "rpicam-still");
		if (!mono)
		{
			TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, CAM_XYZ.m);
			TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, NEUTRAL);
			TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
			TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);

			if (toneCurve.size() > 0)
			{
				// also valid in case of mono?
				TIFFSetField(tif, TIFFTAG_PROFILETONECURVE, uint32_t(toneCurve.size()), toneCurve.data());
			}
		}

		// Make a small greyscale thumbnail, just to give some clue what's in here.
		std::vector<uint8_t> thumb_buf((info.width >> 4) * 3);

		for (unsigned int y = 0; y < (info.height >> 4); y++)
		{
			for (unsigned int x = 0; x < (info.width >> 4); x++)
			{
				unsigned int off = (y * buf_stride_pixels + x) << 4;
				uint32_t grey =
					buf[off] + buf[off + 1] + buf[off + buf_stride_pixels] + buf[off + buf_stride_pixels + 1];
				grey = (grey << 14) >> format.bits;
				grey = sqrt((double)grey); // simple "gamma correction"
				thumb_buf[3 * x] = thumb_buf[3 * x + 1] = thumb_buf[3 * x + 2] = grey;
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
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, mono ? PHOTOMETRIC_LINEARRAW : PHOTOMETRIC_CFA);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(tif, TIFFTAG_WHITELEVEL, 1, &white);
		TIFFSetField(tif, TIFFTAG_NOISEPROFILE, 2, noiseProfile.data());

		if (!mono)
		{
			TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, cfa_repeat_pattern_dim);
#if TIFFLIB_VERSION >= 20201219 // version 4.2.0 or later
			TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, format.order);
#else
			TIFFSetField(tif, TIFFTAG_CFAPATTERN, format.order);
#endif
			const uint16_t black_level_repeat_dim[] = { 2, 2 };
			TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, &black_level_repeat_dim);
			TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 4, &black_levels);

			if (heightLSC > 0 && widthLSC > 0)
			{
				DNGGainMap gainmap0(0, 0, info.height, info.width, 0, 1, 2, 2, heightLSC, widthLSC,
									1.0 / (heightLSC - 1), 1.0 / (widthLSC - 1), 0, 0, 1, lscMaps[format.order[0]]);
				DNGGainMap gainmap1(0, 1, info.height, info.width, 0, 1, 2, 2, heightLSC, widthLSC,
									1.0 / (heightLSC - 1), 1.0 / (widthLSC - 1), 0, 0, 1, lscMaps[format.order[1]]);
				DNGGainMap gainmap2(1, 0, info.height, info.width, 0, 1, 2, 2, heightLSC, widthLSC,
									1.0 / (heightLSC - 1), 1.0 / (widthLSC - 1), 0, 0, 1, lscMaps[format.order[2]]);
				DNGGainMap gainmap3(1, 1, info.height, info.width, 0, 1, 2, 2, heightLSC, widthLSC,
									1.0 / (heightLSC - 1), 1.0 / (widthLSC - 1), 0, 0, 1, lscMaps[format.order[3]]);

				DNGOpcodeList opList;
				opList.AddOpcode(&gainmap0);
				opList.AddOpcode(&gainmap1);
				opList.AddOpcode(&gainmap2);
				opList.AddOpcode(&gainmap3);
				opList.SwitchEndianess();
				auto opListBytes = opList.GetBytes();

				TIFFSetField(tif, TIFFTAG_OPCODELIST2, uint32_t(opListBytes.size()), (char *)opListBytes.data());
			}
		}
		else
		{
			const uint16_t black_level_repeat_dim[] = { 1, 1 };
			TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, &black_level_repeat_dim);
			float black_level = black_levels[0];
			TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 1, &black_level);

			if (heightLSC > 0 && widthLSC > 0)
			{
				DNGGainMap gainmap(0, 0, info.height, info.width, 0, 1, 1, 1, heightLSC, widthLSC,
								   1.0 / (heightLSC - 1), 1.0 / (widthLSC - 1), 0, 0, 1, std::move(lscMaps[1]));
				DNGOpcodeList opList;
				opList.AddOpcode(&gainmap);
				opList.SwitchEndianess();
				auto opListBytes = opList.GetBytes();

				TIFFSetField(tif, TIFFTAG_OPCODELIST3, uint32_t(opListBytes.size()), (char *)opListBytes.data());
			}
		}

		for (unsigned int y = 0; y < info.height; y++)
		{
			if (TIFFWriteScanline(tif, &buf[buf_stride_pixels * y], y, 0) != 1)
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

		auto lp = metadata.get(libcamera::controls::LensPosition);
		if (lp)
		{
			double dist = (*lp > 0.0) ? (1.0 / *lp) : std::numeric_limits<double>::infinity();
			TIFFSetField(tif, EXIFTAG_SUBJECTDISTANCE, dist);
		}

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
