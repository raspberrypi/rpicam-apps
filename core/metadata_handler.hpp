/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * metadata_hander.hpp - class for handling metadata
 */

#pragma once

#include <atomic>
#include <stdio.h>

#include "core/still_options.hpp"
#include "core/video_options.hpp"

class MetadataHandler
{
public:
	MetadataHandler(VideoOptions const *options_);
	void initMetadata(int segmentNum = 0);
	void startMetadataOutput(std::streambuf *buf, std::string fmt);
	void writeMetadata(); //(std::string fmt, libcamera::ControlList &metadata, bool first_write);
	void stopMetadataOutput();
	void MetadataReady(libcamera::ControlList &metadata);
	void setStillMode(StillOptions const *still_options_);
	void discardMetadata();
	void writeStillMetadata(StillOptions const *still_options, libcamera::ControlList &metadata);

protected:
	VideoOptions const *options_;
	StillOptions const *still_options_;
	FILE *fp_timestamps_;

private:
	enum State
	{
		DISABLED = 0,
		WAITING_KEYFRAME = 1,
		RUNNING = 2
	};
	std::streambuf *buf_metadata_;
	std::ofstream of_metadata_;
	std::string fmt_;
	bool first_write_;
	bool video_mode_;
	std::queue<libcamera::ControlList> metadata_queue_;
};
