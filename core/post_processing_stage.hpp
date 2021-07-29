/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * post_processing_stage.hpp - Post processing stage base class definition.
 */

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

class CompletedRequest;
class LibcameraApp;

class PostProcessingStage
{
public:
	PostProcessingStage(LibcameraApp *app);

	virtual ~PostProcessingStage();

	virtual char const *Name() const = 0;

	virtual void Read(boost::property_tree::ptree const &params);

	virtual void Configure();

	virtual void Start();

	virtual void Process(CompletedRequest &completed_request) = 0;

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
