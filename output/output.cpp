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
	: options_(options), fp_timestamps_(nullptr), state_(WAITING_KEYFRAME), time_offset_(0), last_timestamp_(0),
	  buf_metadata_(std::cout.rdbuf()), of_metadata_()
{
	if (!options->Get().save_pts.empty())
	{
		fp_timestamps_ = fopen(options->Get().save_pts.c_str(), "w");
		if (!fp_timestamps_)
			throw std::runtime_error("Failed to open timestamp file " + options->Get().save_pts);
		fprintf(fp_timestamps_, "# timecode format v2\n");
	}
	if (!options->Get().metadata.empty())
	{
		const std::string &filename = options_->Get().metadata;

		if (filename.compare("-"))
		{
			of_metadata_.open(filename, std::ios::out);
			buf_metadata_ = of_metadata_.rdbuf();
			start_metadata_output(buf_metadata_, options_->Get().metadata_format);
		}
	}

	enable_ = !options->Get().pause;
}

Output::~Output()
{
	if (fp_timestamps_)
		fclose(fp_timestamps_);
	if (!options_->Get().metadata.empty())
		stop_metadata_output(buf_metadata_, options_->Get().metadata_format);
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

	if (!options_->Get().metadata.empty())
	{
		libcamera::ControlList metadata = metadata_queue_.front();
		write_metadata(buf_metadata_, options_->Get().metadata_format, metadata, !metadata_started_);
		metadata_started_ = true;
		metadata_queue_.pop();
	}
}

void Output::timestampReady(int64_t timestamp)
{
	fprintf(fp_timestamps_, "%" PRId64 ".%03" PRId64 "\n", timestamp / 1000, timestamp % 1000);
	if (options_->Get().flush)
		fflush(fp_timestamps_);
}

void Output::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	// Supply this so that a vanilla Output gives you an object that outputs no buffers.
}

Output *Output::Create(VideoOptions const *options)
{
	bool libav = options->Get().codec == "libav" ||
				 (options->Get().codec == "h264" && options->GetPlatform() != Platform::VC4);
	const std::string out_file = options->Get().output;

	if (!libav && (strncmp(out_file.c_str(), "udp://", 6) == 0 || strncmp(out_file.c_str(), "tcp://", 6) == 0))
		return new NetOutput(options);
	else if (options->Get().circular)
		return new CircularOutput(options);
	else if (!out_file.empty())
		return new FileOutput(options);
	else
		return new Output(options);
}

void Output::MetadataReady(libcamera::ControlList &metadata)
{
	if (options_->Get().metadata.empty())
		return;

	metadata_queue_.push(metadata);
}

void start_metadata_output(std::streambuf *buf, std::string fmt)
{
	std::ostream out(buf);
	if (fmt == "json")
		out << "[" << std::endl;
}

void write_metadata(std::streambuf *buf, std::string fmt, libcamera::ControlList &metadata, bool first_write)
{
	std::ostream out(buf);
	const libcamera::ControlIdMap *id_map = metadata.idMap();
	if (fmt == "txt")
	{
		for (auto const &[id, val] : metadata)
			out << id_map->at(id)->name() << "=" << val.toString() << std::endl;
		out << std::endl;
	}
	else
	{
		if (!first_write)
			out << "," << std::endl;
		out << "{";
		bool first_done = false;
		for (auto const &[id, val] : metadata)
		{
			std::string arg_quote = (val.toString().find('/') != std::string::npos) ? "\"" : "";
			out << (first_done ? "," : "") << std::endl
				<< "    \"" << id_map->at(id)->name() << "\": " << arg_quote << val.toString() << arg_quote;
			first_done = true;
		}
		out << std::endl << "}";
	}
}

void stop_metadata_output(std::streambuf *buf, std::string fmt)
{
	std::ostream out(buf);
	if (fmt == "json")
		out << std::endl << "]" << std::endl;
}
