/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * post_processing_stage.hpp - Post processing stage base class definition.
 */

#include <string>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace libcamera
{
class StreamConfiguration;
}

class CompletedRequest;
class LibcameraApp;

using StreamConfiguration = libcamera::StreamConfiguration;

class PostProcessingStage
{
public:
	PostProcessingStage(LibcameraApp *app);

	virtual ~PostProcessingStage();

	virtual char const *Name() const = 0;

	virtual void Read(boost::property_tree::ptree const &params);

	virtual void AdjustConfig(std::string const &use_case, StreamConfiguration *config);

	virtual void Configure();

	virtual void Start();

	// Return true if this request is to be dropped.
	virtual bool Process(CompletedRequest &completed_request) = 0;

	virtual void Stop();

	virtual void Teardown();

protected:
	LibcameraApp *app_;
};

typedef PostProcessingStage *(*StageCreateFunc)(LibcameraApp *app);
struct RegisterStage
{
	RegisterStage(char const *name, StageCreateFunc create_func);
};

std::map<std::string, StageCreateFunc> const &GetPostProcessingStages();
