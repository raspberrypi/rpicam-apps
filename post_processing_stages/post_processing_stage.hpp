/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * post_processing_stage.hpp - Post processing stage base class definition.
 */

#include <chrono>
#include <map>
#include <string>

// Prevents compiler warnings in Boost headers with more recent versions of GCC.
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "core/completed_request.hpp"
#include "core/stream_info.hpp"

namespace libcamera
{
struct StreamConfiguration;
}

class RPiCamApp;

using StreamConfiguration = libcamera::StreamConfiguration;

class PostProcessingStage
{
public:
	PostProcessingStage(RPiCamApp *app);

	virtual ~PostProcessingStage();

	virtual char const *Name() const = 0;

	virtual void Read(boost::property_tree::ptree const &params);

	virtual void AdjustConfig(std::string const &use_case, StreamConfiguration *config);

	virtual void Configure();

	virtual void Start();

	// Return true if this request is to be dropped.
	virtual bool Process(CompletedRequestPtr &completed_request) = 0;

	virtual void Stop();

	virtual void Teardown();

	// Below here are some helpers provided for the convenience of derived classes.

	// Convert YUV420 image to RGB. We crop from the centre of the image if the src
	// image is larger than the destination.
	static std::vector<uint8_t> Yuv420ToRgb(const uint8_t *src, StreamInfo &src_info, StreamInfo &dst_info);

protected:
	// Helper to calculate the execution time of any callable object and return it in as a std::chrono::duration.
	// For functions returning a value, the simplest thing would be to wrap the call in a lambda and capture
	// the return value.
	template <class R = std::micro, class T = std::chrono::steady_clock, class F, class... Args>
	static auto ExecutionTime(F &&f, Args &&... args)
	{
		auto t1 = T::now();
		std::invoke(std::forward<decltype(f)>(f), std::forward<Args>(args)...);
		auto t2 = T::now();
		return std::chrono::duration<double, R>(t2 - t1);
	}

	RPiCamApp *app_;
};

typedef PostProcessingStage *(*StageCreateFunc)(RPiCamApp *app);
struct RegisterStage
{
	RegisterStage(char const *name, StageCreateFunc create_func);
};

std::map<std::string, StageCreateFunc> const &GetPostProcessingStages();
