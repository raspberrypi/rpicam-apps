/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * completed_request.hpp - structure holding request results.
 */

#pragma once

#include <libcamera/controls.h>
#include <libcamera/request.h>

#include "core/metadata.hpp"

struct CompletedRequest
{
	using BufferMap = libcamera::Request::BufferMap;
	using ControlList = libcamera::ControlList;

	CompletedRequest() {}

	CompletedRequest(unsigned int seq, BufferMap const &b, ControlList const &m)
		: sequence(seq), buffers(b), metadata(m)
	{
	}
	unsigned int sequence;
	BufferMap buffers;
	ControlList metadata;
	float framerate;
	Metadata post_process_metadata;
};
