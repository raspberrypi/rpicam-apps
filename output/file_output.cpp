/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */
#include <filesystem>
#include <string>

#include "file_output.hpp"
#include "file_name_manager.hpp"
#include "image/image.hpp"
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include "core/still_options.hpp"
#include "core/stream_info.hpp"
#include "core/options.hpp"

namespace fs = std::filesystem;

FileOutput::FileOutput(VideoOptions const *options)
	: Output(options), fp_(nullptr), file_start_time_ms_(0), fileNameManager_((Options*)options)
{
	// Nothing
}

FileOutput::~FileOutput()
{
	closeFile();
}

void FileOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{

	if(options_->force_dng) {
		saveDng(mem);
	} else {
		saveFile(mem, size, timestamp_us, flags);
	}

}

void FileOutput::saveFile(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) {
	// We need to open a new file if we're in "segment" mode and our segment is full
	// (though we have to wait for the next I frame), or if we're in "split" mode
	// and recording is being restarted (this is necessarily an I-frame already).
	if (fp_ == nullptr ||
		(options_->segment && (flags & FLAG_KEYFRAME) &&
		 timestamp_us / 1000 - file_start_time_ms_ > options_->segment) ||
		(options_->split && (flags & FLAG_RESTART)))
	{
		closeFile();
		openFile(timestamp_us);
	}

	LOG(2, "FileOutput: output buffer " << mem << " size " << size);
	if (fp_ && size)
	{
		if (fwrite(mem, size, 1, fp_) != 1)
			throw std::runtime_error("failed to write output bytes");
		if (options_->flush)
			fflush(fp_);
	}
}

void FileOutput::saveDng(void *mem) {
	// TODO figure out how to not mock steamInfo
	libcamera::ControlList mockControlList;

	StreamInfo mockInfo;
	mockInfo.width = 4056;
	mockInfo.height = 3040;
	mockInfo.stride = 6112;
	mockInfo.pixel_format = libcamera::formats::SBGGR12_CSI2P;
	std::string filename = fileNameManager_.getNextFileName();

	// TODO decide on camera name
	dng_save(mem, mockInfo, mockControlList, filename, "mock-camera-model", NULL);
}

void FileOutput::openFile(int64_t timestamp_us)
{
	if (options_->output == "-")
		fp_ = stdout;
	else if (!options_->output.empty())
	{
		std::string filename = fileNameManager_.getNextFileName();
		fp_ = fopen(filename.c_str(), "w");
		if (!fp_)
			throw std::runtime_error("failed to open output file " + std::string(filename));
		LOG(2, "FileOutput: opened output file " << filename);

		file_start_time_ms_ = timestamp_us / 1000;
	}
}

void FileOutput::closeFile()
{
	if (fp_)
	{
		if (options_->flush)
			fflush(fp_);
		if (fp_ != stdout)
			fclose(fp_);
		fp_ = nullptr;
	}
}
