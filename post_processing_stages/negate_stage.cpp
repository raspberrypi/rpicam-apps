/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * negate_stage.cpp - image negate effect
 */

#include <libcamera/stream.h>

#include "../core/libcamera_app.hpp"
#include "../core/post_processing_stage.hpp"

using Stream = libcamera::Stream;

class NegateStage : public PostProcessingStage
{
public:
	NegateStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override {}

	void Configure();

	bool Process(CompletedRequest &completed_request);

private:
	Stream *stream_;
};

#define NAME "negate"

char const *NegateStage::Name() const
{
	return NAME;
}

void NegateStage::Configure()
{
	stream_ = app_->GetMainStream();
}

bool NegateStage::Process(CompletedRequest &completed_request)
{
	int w, h, stride;
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request.buffers[stream_])[0];
	uint8_t *ptr = buffer.data();
	for (unsigned int i = 0; i < buffer.size(); i++)
		*(ptr++) ^= 0xff;
	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new NegateStage(app);
}

static RegisterStage reg(NAME, &Create);
