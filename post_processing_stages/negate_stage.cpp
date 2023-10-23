/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * negate_stage.cpp - image negate effect
 */

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

using Stream = libcamera::Stream;

class NegateStage : public PostProcessingStage
{
public:
	NegateStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override {}

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

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

bool NegateStage::Process(CompletedRequestPtr &completed_request)
{
	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *ptr = (uint32_t *)buffer.data();

	// Constraints on the stride mean we always have multiple-of-4 bytes.
	for (unsigned int i = 0; i < buffer.size(); i += 4)
		*(ptr++) ^= 0xffffffff;

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new NegateStage(app);
}

static RegisterStage reg(NAME, &Create);
