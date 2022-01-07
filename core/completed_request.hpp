/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * completed_request.hpp - structure holding request results.
 */

#pragma once

#include <memory>

#include <libcamera/controls.h>
#include <libcamera/request.h>

#include "core/metadata.hpp"

struct CompletedRequest
{
	using BufferMap = libcamera::Request::BufferMap;
	using ControlList = libcamera::ControlList;
	using Request = libcamera::Request;

	CompletedRequest(unsigned int seq, Request *r)
		: sequence(seq), buffers(r->buffers()), metadata(r->metadata()), request(r)
	{
		r->reuse();
	}
	unsigned int sequence;
	BufferMap buffers;
	ControlList metadata;
	Request *request;
	float framerate;
	Metadata post_process_metadata;
};

using CompletedRequestPtr = std::shared_ptr<CompletedRequest>;
