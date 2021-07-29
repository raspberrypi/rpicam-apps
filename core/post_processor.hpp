/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * post_processor.hpp - Post processor definition.
 */

#pragma once

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <queue>

#include "core/completed_request.hpp"

class LibcameraApp;

using namespace std::chrono_literals;
class PostProcessingStage;
using PostProcessorCallback = std::function<void(CompletedRequest &)>;
typedef std::unique_ptr<PostProcessingStage> StagePtr;

class PostProcessor
{
public:
	PostProcessor(LibcameraApp *app);

	~PostProcessor();

	void Read(std::string const &filename);

	void SetCallback(PostProcessorCallback callback);

	void Configure();

	void Start();

	void Process(CompletedRequest &request);

	void Stop();

	void Teardown();

private:
	PostProcessingStage *createPostProcessingStage(char const *name);

	LibcameraApp *app_;
	std::vector<StagePtr> stages_;
	void outputThread();

	std::queue<CompletedRequest> requests_;
	std::queue<std::future<void>> futures_;
	std::thread output_thread_;
	bool quit_;
	PostProcessorCallback callback_;
	std::mutex mutex_;
	std::condition_variable cv_;
};
