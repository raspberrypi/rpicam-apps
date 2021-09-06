/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * post_processor.cpp - Post processor implementation.
 */

#include <iostream>

#include "core/post_processor.hpp"
#include "post_processing_stage.hpp"
// #include "libcamera_app.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

PostProcessor::PostProcessor(LibcameraApp *app) : app_(app)
{
}

PostProcessor::~PostProcessor()
{
}

void PostProcessor::Read(std::string const &filename)
{
	boost::property_tree::ptree root;
	boost::property_tree::read_json(filename, root);
	for (auto const &key_and_value : root)
	{
		PostProcessingStage *stage = createPostProcessingStage(key_and_value.first.c_str());
		if (stage)
		{
			std::cout << "Reading post processing stage \"" << key_and_value.first << "\"" << std::endl;
			stage->Read(key_and_value.second);
			stages_.push_back(StagePtr(stage));
		}
		else
			std::cout << "No post processing stage found for \"" << key_and_value.first << "\"" << std::endl;
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

void PostProcessor::Process(CompletedRequest &request)
{
	if (stages_.empty())
	{
		callback_(request);
		return;
	}

	std::unique_lock<std::mutex> l(mutex_);
	requests_.push(std::move(request));

	std::promise<void> promise;
	auto process_fn = [this](CompletedRequest &request, std::promise<void> promise) {
		for (auto &stage : stages_)
			stage->Process(request);
		promise.set_value();
		cv_.notify_one();
	};

	// Queue the futures to ensure we have correct ordering in the output thread. The promise/future return value
	// tells us when all the streams for this request have been processed and output_ready_callback_ can be called.
	futures_.push(std::move(promise.get_future()));
	std::thread { process_fn, std::ref(requests_.back()), std::move(promise) }.detach();
}

void PostProcessor::outputThread()
{
	while (true)
	{
		CompletedRequest request;

		{
			std::unique_lock<std::mutex> l(mutex_);

			cv_.wait(l, [this] {
				return (quit_ && futures_.empty()) ||
					   (!futures_.empty() && futures_.front().wait_for(0s) == std::future_status::ready);
			});

			// Only quit when the futures_ queue is empty.
			if (quit_ && futures_.empty())
				break;

			futures_.front().get();
			futures_.pop();
			request = std::move(requests_.front());
			requests_.pop();
		}

		callback_(request);
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
