/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * output.cpp - video stream output base class
 */

#include <cinttypes>
#include <stdexcept>

#include "circular_output.hpp"
#include "file_output.hpp"
#include "net_output.hpp"
#include "output.hpp"

Output::Output(VideoOptions const *options)
	: options_(options), state_(WAITING_KEYFRAME), fp_timestamps_(nullptr), time_offset_(0), last_timestamp_(0)
{
	if (!options->save_pts.empty())
	{
		fp_timestamps_ = fopen(options->save_pts.c_str(), "w");
		if (!fp_timestamps_)
			throw std::runtime_error("Failed to open timestamp file " + options->save_pts);
		fprintf(fp_timestamps_, "# timecode format v2\n");
	}

	enable_ = !options->pause;
}

Output::~Output()
{
	if (fp_timestamps_)
		fclose(fp_timestamps_);
}

void Output::Signal()
{
	enable_ = !enable_;
}

void Output::OutputReady(void *mem, size_t size, int64_t timestamp_us, bool keyframe)
{
	// When output is enabled, we may have to wait for the next keyframe.
	uint32_t flags = keyframe ? FLAG_KEYFRAME : FLAG_NONE;
	if (!enable_)
		state_ = DISABLED;
	else if (state_ == DISABLED)
		state_ = WAITING_KEYFRAME;
	if (state_ == WAITING_KEYFRAME && keyframe)
		state_ = RUNNING, flags |= FLAG_RESTART;
	if (state_ != RUNNING)
		return;

	// Frig the timestamps to be continuous after a pause.
	if (flags & FLAG_RESTART)
		time_offset_ = timestamp_us - last_timestamp_;
	last_timestamp_ = timestamp_us - time_offset_;

	outputBuffer(mem, size, last_timestamp_, flags);

	// Save timestamps to a file, if that was requested.
	if (fp_timestamps_)
		fprintf(fp_timestamps_, "%" PRId64 ".%03" PRId64 "\n", last_timestamp_ / 1000, last_timestamp_ % 1000);
}

void Output::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	// Supply this so that a vanilla Output gives you an object that outputs no buffers.
}

Output *Output::Create(VideoOptions const *options)
{
	if (strncmp(options->output.c_str(), "udp://", 6) == 0 || strncmp(options->output.c_str(), "tcp://", 6) == 0)
		return new NetOutput(options);
	else if (options->circular)
		return new CircularOutput(options);
	else if (!options->output.empty())
		return new FileOutput(options);
	else
		return new Output(options);
}
