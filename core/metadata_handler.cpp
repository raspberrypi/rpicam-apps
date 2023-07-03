/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * metadata_hander.cpp - class for handling metadata
 */

#include "metadata_handler.hpp"

MetadataHandler::MetadataHandler(VideoOptions const *options)
	: options_(options), still_options_(nullptr), buf_metadata_(std::cout.rdbuf()), of_metadata_(), fmt_(),
	  first_write_(true), video_mode_(true)
{
	if (options != nullptr)
		fmt_ = options_->metadata_format;
}

void MetadataHandler::initMetadata(int segmentNum)
{
	std::string filename = "";
	if (video_mode_)
	{
		filename = options_->metadata;
		if (options_->split || options_->segment)
		{
			// If in split/segment mode we need to generate filename
			char subfilename[256];
			int n;
			n = snprintf(subfilename, sizeof(subfilename), options_->metadata.c_str(), segmentNum);
			// Turns the %d into useful numbers
			if (n < 0)
				throw std::runtime_error("failed to generate metadata filename");
			filename = subfilename;
		}
	}
	else
		filename = still_options_->metadata;
	of_metadata_.open(filename, std::ios::out);
	buf_metadata_ = of_metadata_.rdbuf();
	first_write_ = true;
	startMetadataOutput(buf_metadata_, fmt_);
}

void MetadataHandler::setStillMode(StillOptions const *still_options)
{
	// This is used to supply MetadataHandler with options for it to work in still mode
	// Metadata handler defaults to being setup for video mode
	still_options_ = still_options;
	video_mode_ = false;
	fmt_ = still_options_->metadata_format;
}

void MetadataHandler::startMetadataOutput(std::streambuf *buf, std::string fmt)
{
	// create the heading for the file
	std::ostream out(buf_metadata_);
	if (fmt == "json")
		out << "[" << std::endl;
}

void MetadataHandler::writeMetadata()
{
	// Takes metadata from the queue, writes to previously specified file
	if (!metadata_queue_.empty())
	{
		libcamera::ControlList metadata = metadata_queue_.front();
		metadata_queue_.pop();
		std::ostream out(buf_metadata_);
		const libcamera::ControlIdMap *id_map = metadata.idMap();
		if (fmt_ == "txt")
		{
			for (auto const &[id, val] : metadata)
				out << id_map->at(id)->name() << "=" << val.toString() << std::endl;
			out << std::endl;
		}
		else
		{
			if (!first_write_)
				out << "," << std::endl;
			first_write_ = false;
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
}

void MetadataHandler::stopMetadataOutput()
{
	// Closes down the file & adds closing statement for (optional) json format.
	std::ostream out(buf_metadata_);
	if (fmt_ == "json")
		out << std::endl << "]" << std::endl;
	of_metadata_.close();
	std::queue<libcamera::ControlList>().swap(metadata_queue_);
}

void MetadataHandler::MetadataReady(libcamera::ControlList &metadata)
{
	// function that accepts metadata from the encode feed, and pushes it onto the queue
	if (video_mode_ && options_->metadata.empty())
		return;
	metadata_queue_.push(metadata);
}

void MetadataHandler::discardMetadata()
{
	// Useful when discarding a frame due to pausing or unpausing
	metadata_queue_.pop();
}

void MetadataHandler::writeStillMetadata(StillOptions const *still_options, libcamera::ControlList &metadata)
{
	// Created for simplicity when writing metadata for still files
	setStillMode(still_options);
	initMetadata(0);
	MetadataReady(metadata);
	writeMetadata();
	stopMetadataOutput();
}
