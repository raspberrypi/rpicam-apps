/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * post_processing_stage.cpp - Post processing stage base class implementation.
 */

#include "core/post_processing_stage.hpp"

PostProcessingStage::PostProcessingStage(LibcameraApp *app) : app_(app)
{
}

PostProcessingStage::~PostProcessingStage()
{
}

void PostProcessingStage::Read(boost::property_tree::ptree const &params)
{
}

void PostProcessingStage::AdjustConfig(std::string const &, StreamConfiguration *)
{
}

void PostProcessingStage::Configure()
{
}

void PostProcessingStage::Start()
{
}

// Process is pure virtual.

void PostProcessingStage::Stop()
{
}

void PostProcessingStage::Teardown()
{
}

static std::map<std::string, StageCreateFunc> stages;
std::map<std::string, StageCreateFunc> const &GetPostProcessingStages()
{
	return stages;
}

RegisterStage::RegisterStage(char const *name, StageCreateFunc create_func)
{
	stages[std::string(name)] = create_func;
}
