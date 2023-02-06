/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include "core/still_options.hpp"
#include "core/stream_info.hpp"

#include "dng_encoder.hpp"
#include <tiffio.h>
#include <arm_neon.h>
#include "lj92.h"
#include "utils.hpp"

#include "yuv2rgb.hpp"

#include <filesystem>
namespace fs = std::filesystem;

using namespace libcamera;

static char TIFF_RGGB[4] = { 0, 1, 1, 2 };
static char TIFF_GRBG[4] = { 1, 0, 2, 1 };
static char TIFF_BGGR[4] = { 2, 1, 1, 0 };
static char TIFF_GBRG[4] = { 1, 2, 0, 1 };

struct BayerFormat
{
	const char *name;
	int bits;
	const char *order;
};

ttag_t TIFFTAG_FRAMERATE =  0xC764;
ttag_t TIFFTAG_TIMECODE = 0xC763;
static const TIFFFieldInfo xtiffFieldInfo[] = {
    { TIFFTAG_FRAMERATE, 1, 1, TIFF_RATIONAL,	FIELD_CUSTOM,
      true,	false,	"FrameRate" },
    { TIFFTAG_TIMECODE,	8, 8, TIFF_BYTE,	FIELD_CUSTOM,
      true,	false,	"TimeCodes" },
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
	{ formats::SBGGR12, { "BGGR-12", 12, TIFF_BGGR } },
	{ formats::SBGGR10, { "BGGR-10", 10, TIFF_BGGR } },
};

extern __attribute__((noinline, section("disasm"))) void unpack12p(uint8x16x3_t *input){
    uint8x16_t tmp1 = input->val[1];
    uint8x16_t tmp2 = input->val[2];

    input->val[1] = vorrq_u8(vshlq_n_u8(tmp2,4),vshrq_n_u8(tmp1,4));
    input->val[2] = vorrq_u8(vshlq_n_u8(tmp1,4),vshrq_n_u8(tmp2,4));
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

DngEncoder::DngEncoder(RawOptions const *options)
	: Encoder(options), abortEncode_(false), abortOutput_(false), index_(0), frameStop_(0), frames_(0), resetCount_(false), encodeCheck_(false), cache_buffer_(448), compressed(false)
{
    options_ = options;
	// output_thread_ = std::thread(&DngEncoder::outputThread, this);
	for (int i = 0; i < NUM_ENC_THREADS; i++){
		encode_thread_[i] = std::thread(std::bind(&DngEncoder::encodeThread, this, i));
		cache_thread_[i] = std::thread(std::bind(&DngEncoder::cacheThread, this, i));
	}
	LOG(2, "Opened DngEncoder");
}

DngEncoder::~DngEncoder()
{
	abortEncode_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++){
		encode_thread_[i].join();
		cache_thread_[i].join();
	}
	abortOutput_ = true;
	// output_thread_.join();
	LOG(2, "DngEncoder closed");
}

void DngEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
{
	{
		std::lock_guard<std::mutex> lock(encode_mutex_);
	}
    {
        input_done_callback_(nullptr);
        output_ready_callback_(mem, size, timestamp_us, true);
	}		
}

void DngEncoder::EncodeBuffer2(int fd, size_t size, void *mem, StreamInfo const &info, size_t losize, void *lomem, StreamInfo const &loinfo, int64_t timestamp_us, CompletedRequest::ControlList const &metadata)
{
	{
		std::lock_guard<std::mutex> lock(encode_mutex_);
		EncodeItem item = { mem, size, info, lomem, losize, loinfo, metadata, timestamp_us, index_++ };
		encode_queue_.push(item);
		encode_cond_var_.notify_all();
	}
}

void DngEncoder::dng_save(uint8_t const *mem, StreamInfo const &info, uint8_t const *lomem, StreamInfo const &loinfo, size_t losize,
			  ControlList const &metadata, std::string const &filename,
			  std::string const &cam_name, RawOptions const *options, uint64_t fn)
{
	
	uint8_t rawUniq[8]; 
	memset(rawUniq, 0, sizeof(rawUniq));
	auto rU = metadata.get(libcamera::controls::SensorTimestamp);
	if(rU){
		memcpy(rawUniq, (uint8_t*)&(*rU), sizeof(rawUniq));
	}

	// Check the Bayer format
	auto it = bayer_formats.find(info.pixel_format);
	if (it == bayer_formats.end())
		throw std::runtime_error("unsupported Bayer format");
	BayerFormat const &bayer_format = it->second;
	LOG(2, "Bayer format is " << bayer_format.name);

	// We need to fish out some metadata values for the DNG.
	float black = 4096 * (1 << bayer_format.bits) / 65536.0;
	float black_levels[] = { black, black, black, black };
	auto bl = metadata.get(controls::SensorBlackLevels);
	if (bl)
	{
		// levels is in the order R, Gr, Gb, B. Re-order it for the actual bayer order.
		for (int i = 0; i < 4; i++)
		{
			int j = bayer_format.order[i];
			j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!bayer_format.order[i ^ 1]);
			black_levels[j] = (*bl)[i] * (1 << bayer_format.bits) / 65536.0;
		}
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

	float NEUTRAL[] = { 1, 1, 1 };
	float ANALOGBALANCE[] = { 1, 1, 1 };
	Matrix WB_GAINS(1, 1, 1);
	auto cg = metadata.get(controls::ColourGains);
	if (cg)
	{
		NEUTRAL[0] = 1.0 / (*cg)[0];
		NEUTRAL[2] = 1.0 / (*cg)[1];
		WB_GAINS = Matrix((*cg)[0], 1, (*cg)[1]);
	}

	// Use a slightly plausible default CCM in case the metadata doesn't have one (it should!).
	Matrix CCM(1.90255, -0.77478, -0.12777,
			   -0.31338, 1.88197, -0.56858,
			   -0.06001, -0.61785, 1.67786);
	auto ccm = metadata.get(controls::ColourCorrectionMatrix);
	if (ccm)
	{
		CCM = Matrix((*ccm)[0], (*ccm)[1], (*ccm)[2], (*ccm)[3], (*ccm)[4], (*ccm)[5], (*ccm)[6], (*ccm)[7], (*ccm)[8]);
	}
	else
		LOG_ERROR("WARNING: no CCM metadata found");

	// This maxtrix from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
	Matrix RGB2XYZ(0.4124564, 0.3575761, 0.1804375,
				   0.2126729, 0.7151522, 0.0721750,
				   0.0193339, 0.1191920, 0.9503041);
	Matrix CAM_XYZ = (RGB2XYZ * CCM * WB_GAINS).Inv();

	// LOG(2, "Black levels " << black_levels[0] << " " << black_levels[1] << " " << black_levels[2] << " "
	// 					   << black_levels[3] << ", exposure time " << exp_time * 1e6 << "us, ISO " << iso);
	// LOG(2, "Neutral " << NEUTRAL[0] << " " << NEUTRAL[1] << " " << NEUTRAL[2]);
	// LOG(2, "Cam_XYZ: ");
	// LOG(2, CAM_XYZ.m[0] << " " << CAM_XYZ.m[1] << " " << CAM_XYZ.m[2]);
	// LOG(2, CAM_XYZ.m[3] << " " << CAM_XYZ.m[4] << " " << CAM_XYZ.m[5]);
	// LOG(2, CAM_XYZ.m[6] << " " << CAM_XYZ.m[7] << " " << CAM_XYZ.m[8]);

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
		TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, loinfo.width);
		TIFFSetField(tif, TIFFTAG_IMAGELENGTH, loinfo.height);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_MAKE, "Raspberry Pi");
		TIFFSetField(tif, TIFFTAG_MODEL, options_->sensor.c_str());
		TIFFSetField(tif, TIFFTAG_DNGVERSION, "\001\004\000\000");
		TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, "\001\001\000\000");
		TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, options_->serial.c_str());
		TIFFSetField(tif, TIFFTAG_RAWDATAUNIQUEID, rawUniq);
		TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(tif, TIFFTAG_SOFTWARE, "Libcamera;cinepi-raw");
		TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, CAM_XYZ.m);
		TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, NEUTRAL);
		TIFFSetField(tif, TIFFTAG_CALIBRATIONILLUMINANT1, 21);
		TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
		TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);

		// LOG(1, losize << " , " << loinfo.width << " , " << loinfo.height << " , " << loinfo.stride);

		size_t rowSize = loinfo.stride*3;
		size_t thumbSize = rowSize*loinfo.height;
		uint8_t *thumb = (uint8_t*)malloc(thumbSize);
		uint8_t *read = &thumb[0];
		if(nv21_to_rgb(thumb, lomem, loinfo.stride, loinfo.height) != 1){
			throw std::runtime_error("error converting yuv2rgb image data");
		}
		for(int y = 0; y < loinfo.height; y++){
			if (TIFFWriteScanline(tif, (read + y*rowSize), y, 0) != 1)
				throw std::runtime_error("error writing DNG image data");
			
		}
		free(thumb);
		TIFFWriteDirectory(tif);

		// LOG(1, "w: " << info.width << ", " << info.height);

		// The main image (actually tends to show up as "sub-image 1").
		TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
		if(info.pixel_format == formats::SBGGR12 || info.pixel_format == formats::SBGGR10){
			TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, info.stride / 2);
		} else {
			TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, info.width);
		}
		TIFFSetField(tif, TIFFTAG_IMAGELENGTH, info.height);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bayer_format.bits);
        TIFFSetField(tif, TIFFTAG_COMPRESSION, options_->compression);
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
		
		TIFFSetField(tif, TIFFTAG_ANALOGBALANCE, 3, ANALOGBALANCE);
		TIFFSetField(tif, TIFFTAG_BASELINEEXPOSURE, 1.0);
		TIFFSetField(tif, TIFFTAG_BASELINENOISE, 1.0);
		TIFFSetField(tif, TIFFTAG_BASELINESHARPNESS, 1.0);
		TIFFSetField(tif, TIFFTAG_BAYERGREENSPLIT, 0);
		TIFFSetField(tif, TIFFTAG_LINEARRESPONSELIMIT, 1.0);


		// const float scale[] = { 1.0, 1.0 };
		// TIFFSetField(tif, TIFFTAG_DEFAULTSCALE, &scale);
		// const float origin[] = { 2.0, 0.0 };
		// TIFFSetField(tif, TIFFTAG_DEFAULTCROPORIGIN, &origin);
		// const float dsize[] = { (float)2020, (float)info.height };
		// TIFFSetField(tif, TIFFTAG_DEFAULTCROPSIZE, &dsize);
		// const uint32_t area[] = { 0, 0 ,(uint32_t)info.height, (uint32_t)(info.stride / 2) };
		// TIFFSetField(tif, TIFFTAG_ACTIVEAREA, &area);
		
		time_t t;
		time(&t);
		struct tm *time_info = localtime(&t);
		TIFFMergeFieldInfo(tif, xtiffFieldInfo, 2);
		const double frameRate = (double)*options_->framerate;
		TIFFSetField(tif, TIFFTAG_FRAMERATE, &frameRate);
		const char tiemcode[] = { (uint8_t)(fn % (uint8_t)frameRate),time_info->tm_sec,time_info->tm_min, time_info->tm_hour, 0, 0, 0, 0 };
		TIFFSetField(tif, TIFFTAG_TIMECODE, &tiemcode);

		bool uncompressed = (info.pixel_format != formats::SBGGR12 || info.pixel_format != formats::SBGGR10 );

		if(uncompressed && options_->compression == COMPRESSION_NONE){
			// NEON UNPACK
			uint8x16x3_t nbuf;
			uint8x16x3_t nbuf1;
			uint8x16x3_t nbuf2;
			uint8x16x3_t nbuf3;

			auto start_time = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> encode_time(0);
			for (unsigned int y = 0; y < info.height; y++)
			{
				uint64_t buffer[384];
				uint64_t* read = (uint64_t *)(mem + y*info.stride);

				for(uint64_t* write = buffer; write < buffer + info.stride/8; write += 24){
					nbuf = vld3q_u8((uint8_t const*)read);
					nbuf1 = vld3q_u8((uint8_t const*)read+48);
					nbuf2 = vld3q_u8((uint8_t const*)read+96);
					nbuf3 = vld3q_u8((uint8_t const*)read+144);
					unpack12p(&nbuf);
					unpack12p(&nbuf1);
					unpack12p(&nbuf2);
					unpack12p(&nbuf3);
					vst3q_u8((uint8_t *)write, nbuf);
					vst3q_u8((uint8_t *)write+48, nbuf1);
					vst3q_u8((uint8_t *)write+96, nbuf2);
					vst3q_u8((uint8_t *)write+144, nbuf3);
					read += 24;
				}
				encode_time += (std::chrono::high_resolution_clock::now() - start_time);

				if (TIFFWriteScanline(tif, (uint8_t *)buffer, y, 0) != 1)
					throw std::runtime_error("error writing DNG image data");
			}
			LOG(2, "unpack in: " << (encode_time.count()) << "ms");
			// END NEON UNPACK
		} else if(options_->compression == COMPRESSION_JPEG){
			// LJ92 START
			uint8_t *encoded = NULL;
			int encodedLength;
			int w, h;
			w = info.stride / 2;
			h = info.height;
			auto start_time = std::chrono::high_resolution_clock::now();
			int ret = lj92_encode((uint16_t*)mem,w*2,(h/2),16,w*h,0,NULL,0,&encoded,&encodedLength);
			auto end_time = (std::chrono::high_resolution_clock::now() - start_time);
			auto duration(std::chrono::duration_cast<std::chrono::milliseconds>(end_time));
			if(ret == LJ92_ERROR_NONE){
				TIFFWriteRawStrip(tif, 0, &encoded[0], encodedLength);
			} else {
				throw std::runtime_error("LJ92 Failed!");
			}
			free(encoded);
			//LJ92 END
			// LOG(1, "SIZE: " << encodedLength << " in :" << duration.count());
		}

		TIFFCheckpointDirectory(tif);
		offset_subifd = TIFFCurrentDirOffset(tif);
		TIFFWriteDirectory(tif);

		TIFFCreateEXIFDirectory(tif);
		char time_str[32];
		strftime(time_str, 32, "%Y:%m:%d %H:%M:%S", time_info);
		TIFFSetField(tif, EXIFTAG_DATETIMEORIGINAL, time_str);

		TIFFSetField(tif, EXIFTAG_ISOSPEEDRATINGS, 1, &iso);
		TIFFSetField(tif, EXIFTAG_EXPOSURETIME, exp_time);

		TIFFCheckpointDirectory(tif);
		offset_exififd = TIFFCurrentDirOffset(tif);
		TIFFWriteDirectory(tif);

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


void DngEncoder::encodeThread(int num)
{
	std::chrono::duration<double> encode_time(0);
	EncodeItem encode_item;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(encode_mutex_);
			while (true)
			{	

				using namespace std::chrono_literals;
				if (abortEncode_ && encode_queue_.empty())
				{
					if (frames_)
						LOG(2, "Encode " << frames_ << " frames, average time " << encode_time.count() * 1000 / frames_
										 << "ms");
					return;
				}
				if (!encode_queue_.empty())
				{
					encode_item = encode_queue_.front();
					encode_queue_.pop();
					break;
				}
				else{
					encode_cond_var_.wait_for(lock, 200ms);
				}
			}
		}

		frames_ = {encode_item.index};
		LOG(1, "memcpy frame: " << encode_item.index);

		{	
			uint8_t *mem = (uint8_t*)malloc(encode_item.size);
			memcpy(mem, encode_item.mem, encode_item.size);
			uint8_t *lomem = (uint8_t*)malloc(encode_item.losize);
			memcpy(lomem, encode_item.lomem, encode_item.losize);
			CachedItem item = { mem, encode_item.size, encode_item.info, lomem, encode_item.losize, encode_item.loinfo, encode_item.met, encode_item.timestamp_us, encode_item.index };
			std::lock_guard<std::mutex> lock(cache_mutex_);
			cache_buffer_.push_back(std::move(item));
			cache_cond_var_.notify_all();
		}

		{
			input_done_callback_(nullptr);
			output_ready_callback_(encode_item.mem, encode_item.size, encode_item.timestamp_us, true);
		}		

	}
}

void DngEncoder::cacheThread(int num)
{
	std::chrono::duration<double> cache_time(0);
	CachedItem cache_item;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(cache_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				if (abortEncode_ && cache_buffer_.empty())
				{
					if (frames_)
						LOG(2, "Encode " << frames_ << " frames, average time " << cache_time.count() * 1000 / frames_
										 << "ms");
					return;
				}
				if (!cache_buffer_.empty())
				{
					cache_item = cache_buffer_.front();
					cache_buffer_.pop_front();
					break;
				}
				else{
					cache_cond_var_.wait_for(lock, 200ms);
				}
			}
		}

		char ft[128];
		if(still_capture){
			snprintf(ft, sizeof(ft), "%s/%s/%s_%09ld.dng", options_->mediaDest.c_str(), std::string("stills").c_str(), options_->folder.c_str(), cache_item.index);
		} else {
			snprintf(ft, sizeof(ft), "%s/%s/%s_%09ld.dng", options_->mediaDest.c_str(), options_->folder.c_str(), options_->folder.c_str(), cache_item.index);
		}
		
		std::string filename = std::string(ft);
		LOG(1, "save frame: " << cache_item.index);

		bool dm = disk_mounted(options_);
		auto start_time = std::chrono::high_resolution_clock::now();
		if(dm){
			dng_save((const uint8_t*)cache_item.mem, cache_item.info, (const uint8_t*)cache_item.lomem, cache_item.loinfo, cache_item.losize, cache_item.met, filename, "CINEPI-2K", options_, cache_item.index);
		}
		auto end_time = (std::chrono::high_resolution_clock::now() - start_time);

		still_capture = false;
		
		cache_time += (end_time);
		free(cache_item.mem);
		free(cache_item.lomem);
	}
}
