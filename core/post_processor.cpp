/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * post_processor.cpp - Post processor implementation.
 */

#include <dlfcn.h>
#include <filesystem>
#include <iostream>
#include <map>

#include "core/options.hpp"
#include "core/rpicam_app.hpp"
#include "core/post_processor.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <libcamera/formats.h>

#include "post_processing_stages/postproc_lib.h"

namespace fs = std::filesystem;

PostProcessingLib::PostProcessingLib(const std::string &lib)
{
	if (!lib.empty())
	{
		lib_ = dlopen(lib.c_str(), RTLD_LAZY);
		if (!lib_)
			LOG_ERROR("Unable to open " << lib << " with error: " << dlerror());
	}
}

PostProcessingLib::PostProcessingLib(PostProcessingLib &&other)
{
	lib_ = other.lib_;
	symbol_map_ = std::move(other.symbol_map_);
	other.lib_ = nullptr;
}

PostProcessingLib::~PostProcessingLib()
{
	if (lib_)
		dlclose(lib_);
}

const void *PostProcessingLib::GetSymbol(const std::string &symbol)
{
	if (!lib_)
		return nullptr;

	std::scoped_lock<std::mutex> l(lock_);

	const auto it = symbol_map_.find(symbol);
	if (it == symbol_map_.end())
	{
		const void *fn = dlsym(lib_, symbol.c_str());

		if (!fn)
		{
			LOG_ERROR("Unable to find postprocessing symbol " << symbol << " with error: " << dlerror());
			return nullptr;
		}

		symbol_map_[symbol] = fn;
	}

	return symbol_map_[symbol];
}

PostProcessor::PostProcessor(RPiCamApp *app) : app_(app)
{
}

PostProcessor::~PostProcessor()
{
	// Must clear stages_ before dynamic_stages_ as the latter will unload the necessary symbols.
	stages_.clear();
	dynamic_stages_.clear();
}

void PostProcessor::LoadModules(const std::string &lib_dir)
{
	const fs::path path(!lib_dir.empty() ? lib_dir : POSTPROC_LIB_DIR);
	const std::string ext(".so");

	if (!fs::exists(path))
		return;

	// Dynamically load all .so files from the system postprocessing lib path.
	// This will automatically register the stages with the factory.
	for (auto const &p : fs::recursive_directory_iterator(path))
	{
		if (p.path().extension() == ext)
			dynamic_stages_.emplace_back(p.path().string());
	}
}

void PostProcessor::Read(std::string const &filename)
{
	boost::property_tree::ptree root;
	boost::property_tree::read_json(filename, root);
	for (auto const &key_and_value : root)
	{
		if (key_and_value.first == "rpicam-apps")
		{
			boost::property_tree::ptree const &node = key_and_value.second;

			if (node.find("lores") != node.not_found())
			{
				static std::map<std::string, libcamera::PixelFormat> formats {
					{ "rgb", libcamera::formats::BGR888 },
					{ "bgr", libcamera::formats::RGB888 },
					{ "yuv420", libcamera::formats::YUV420 },
				};

				unsigned int lores_width = node.get<unsigned int>("lores.width");
				unsigned int lores_height = node.get<unsigned int>("lores.height");
				bool lores_par = node.get<bool>("lores.par", app_->GetOptions()->lores_par);
				std::string lores_format_str = node.get<std::string>("lores.format", "yuv420");

				libcamera::PixelFormat lores_format = libcamera::formats::YUV420;

				auto it = formats.find(lores_format_str);
				if (it == formats.end())
					LOG_ERROR("Unknown requested lores format: " << lores_format_str);
				else
					lores_format = it->second;

				app_->GetOptions()->lores_width = lores_width;
				app_->GetOptions()->lores_height = lores_height;
				app_->GetOptions()->lores_par = lores_par;
				app_->lores_format_ = lores_format;

				LOG(1, "Postprocessing requested lores: " << lores_width << "x" << lores_height << " " << lores_format);
			}
		}
		else
		{
			PostProcessingStage *stage = createPostProcessingStage(key_and_value.first.c_str());
			if (stage)
			{
				LOG(1, "Reading post processing stage \"" << key_and_value.first << "\"");
				stage->Read(key_and_value.second);
				stages_.push_back(StagePtr(stage));
			}
			else
				LOG(1, "No post processing stage found for \"" << key_and_value.first << "\"");
		}
	}
}

PostProcessingStage *PostProcessor::createPostProcessingStage(char const *name)
{
	auto it = GetPostProcessingStages().find(std::string(name));
	return it != GetPostProcessingStages().end() ? (*it->second)(app_) : nullptr;
}

void PostProcessor::SetCallback(PostProcessorCallback callback)
{
	callback_ = callback;
}

void PostProcessor::AdjustConfig(std::string const &use_case, StreamConfiguration *config)
{
	for (auto &stage : stages_)
	{
		stage->AdjustConfig(use_case, config);
	}
}

void PostProcessor::Configure()
{
	for (auto &stage : stages_)
	{
		stage->Configure();
	}
}

void PostProcessor::Start()
{
	quit_ = false;
	output_thread_ = std::thread(&PostProcessor::outputThread, this);

	for (auto &stage : stages_)
	{
		stage->Start();
	}
}

void PostProcessor::Process(CompletedRequestPtr &request)
{
	if (stages_.empty())
	{
		callback_(request);
		return;
	}

	std::unique_lock<std::mutex> l(mutex_);
	requests_.push(std::move(request)); // caller has given us ownership of this reference

	std::promise<bool> promise;
	auto process_fn = [this](CompletedRequestPtr &request, std::promise<bool> promise) {
		bool drop_request = false;
		for (auto &stage : stages_)
		{
			if (stage->Process(request))
			{
				drop_request = true;
				break;
			}
		}
		promise.set_value(drop_request);
		cv_.notify_one();
	};

	// Queue the futures to ensure we have correct ordering in the output thread. The promise/future return value
	// tells us when all the streams for this request have been processed and output_ready_callback_ can be called.
	futures_.push(promise.get_future());
	std::thread { process_fn, std::ref(requests_.back()), std::move(promise) }.detach();
}

void PostProcessor::outputThread()
{
	while (true)
	{
		CompletedRequestPtr request;

		bool drop_request = false;
		{
			std::unique_lock<std::mutex> l(mutex_);

			cv_.wait(l, [this] {
				return (quit_ && futures_.empty()) ||
					   (!futures_.empty() && futures_.front().wait_for(0s) == std::future_status::ready);
			});

			// Only quit when the futures_ queue is empty.
			if (quit_ && futures_.empty())
				break;

			drop_request = futures_.front().get();
			futures_.pop();
			request = std::move(requests_.front()); // reuse as it's being dropped from the queue
			requests_.pop();
		}

		if (!drop_request)
			callback_(request); // callback can take over ownership from us
	}
}

void PostProcessor::Stop()
{
	for (auto &stage : stages_)
	{
		stage->Stop();
	}

	{
		std::unique_lock<std::mutex> l(mutex_);
		quit_ = true;
		cv_.notify_one();
	}

	output_thread_.join();
}

void PostProcessor::Teardown()
{
	for (auto &stage : stages_)
	{
		stage->Teardown();
	}
}
