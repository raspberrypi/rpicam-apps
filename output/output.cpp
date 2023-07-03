/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * output.cpp - video stream output base class
 */

#include <cinttypes>
#include <stdexcept>

#include "circular_output.hpp"
#include "core/metadata_handler.hpp"
#include "file_output.hpp"
#include "net_output.hpp"
#include "output.hpp"

Output::Output(VideoOptions const *options)
	: options_(options), fp_timestamps_(nullptr), state_(WAITING_KEYFRAME), time_offset_(0), last_timestamp_(0),
	  segment_num_(0), previous_segment_num_(0), metadata_handler_(options)
{
	if (!options->save_pts.empty())
	{
		fp_timestamps_ = fopen(options->save_pts.c_str(), "w");
		if (!fp_timestamps_)
			throw std::runtime_error("Failed to open timestamp file " + options->save_pts);
		fprintf(fp_timestamps_, "# timecode format v2\n");
	}
	if (!options->metadata.empty())
		metadata_handler_.initMetadata(0);
	enable_ = !options->pause;
}

Output::~Output()
{
	if (fp_timestamps_)
		fclose(fp_timestamps_);
	if (!options_->metadata.empty())
		metadata_handler_.stopMetadataOutput();
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
	{
		timestampReady(last_timestamp_);
	}

	segment_num_ = getSegmentNum();
	if ((segment_num_ != previous_segment_num_) && (!options_->save_pts.empty()))
	{
		// When we detect that a new file has been made for outputting, we must create new timestamp file
		std::string filename = options_->save_pts.c_str();
		char subfilename[256];
		int n;
		n = snprintf(subfilename, sizeof(subfilename), options_->save_pts.c_str(), segment_num_);
		// Turns the %d into useful numbers
		if (n < 0)
			throw std::runtime_error("failed to generate timestamp filename");

		fp_timestamps_ = fopen(subfilename, "w"); // Updates the filepath for the timestamper to use
		if (!fp_timestamps_)
			throw std::runtime_error("Failed to open timestamp file " + filename);
		fprintf(fp_timestamps_, "# timecode format v2\n");
		time_offset_ = timestamp_us; // Updates time since segment start
		last_timestamp_ = timestamp_us - time_offset_;
	}

	// Code for adding metadata below:
	if (!options_->metadata.empty() && (segment_num_ != previous_segment_num_))
	{
		metadata_handler_.stopMetadataOutput();
		metadata_handler_.initMetadata(segment_num_);
	}

	if (!options_->metadata.empty())
	{
		// write some metadata
		metadata_handler_.writeMetadata();
	}
	previous_segment_num_ = segment_num_;
}

void Output::timestampReady(int64_t timestamp)
{
	fprintf(fp_timestamps_, "%" PRId64 ".%03" PRId64 "\n", timestamp / 1000, timestamp % 1000);
	if (options_->flush)
		fflush(fp_timestamps_);
}

void Output::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	// Supply this so that a vanilla Output gives you an object that outputs no buffers.
}

Output *Output::Create(VideoOptions const *options)
{
	if (options->codec == "libav")
		return new Output(options);

	if (strncmp(options->output.c_str(), "udp://", 6) == 0 || strncmp(options->output.c_str(), "tcp://", 6) == 0)
		return new NetOutput(options);
	else if (options->circular)
		return new CircularOutput(options);
	else if (!options->output.empty())
		return new FileOutput(options);
	else
		return new Output(options);
}

int Output::getSegmentNum()
{
	return 0;
	// Overridden by fileoutput to get the segment number currently on
}

void Output::MetadataReady(libcamera::ControlList &metadata)
{
	if (enable_)
		metadata_handler_.MetadataReady(metadata);
}
