/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * output.hpp - video stream output base class
 */

#pragma once

#include <cstdio>

#include <atomic>

#include "core/video_options.hpp"

class Output
{
public:
	static Output *Create(VideoOptions const *options);

	Output(VideoOptions const *options);
	virtual ~Output();
	virtual void Signal(); // a derived class might redefine what this means
	void OutputReady(void *mem, size_t size, int64_t timestamp_us, bool keyframe);
	void MetadataReady(libcamera::ControlList &metadata);

protected:
	enum Flag
	{
		FLAG_NONE = 0,
		FLAG_KEYFRAME = 1,
		FLAG_RESTART = 2
	};
	virtual void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags);
	virtual void timestampReady(int64_t timestamp);
	VideoOptions const *options_;
	FILE *fp_timestamps_;

private:
	enum State
	{
		DISABLED = 0,
		WAITING_KEYFRAME = 1,
		RUNNING = 2
	};
	State state_;
	std::atomic<bool> enable_;
	int64_t time_offset_;
	int64_t last_timestamp_;
	std::streambuf *buf_metadata_;
	std::ofstream of_metadata_;
	bool metadata_started_ = false;
	std::queue<libcamera::ControlList> metadata_queue_;
};

void start_metadata_output(std::streambuf *buf, std::string fmt);
void write_metadata(std::streambuf *buf, std::string fmt, libcamera::ControlList &metadata, bool first_write);
void stop_metadata_output(std::streambuf *buf, std::string fmt);
