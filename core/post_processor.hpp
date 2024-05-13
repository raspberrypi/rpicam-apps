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
#include "core/logging.hpp"

namespace libcamera
{
struct StreamConfiguration;
}

class RPiCamApp;

using namespace std::chrono_literals;
class PostProcessingStage;
using PostProcessorCallback = std::function<void(CompletedRequestPtr &)>;
using StreamConfiguration = libcamera::StreamConfiguration;
typedef std::unique_ptr<PostProcessingStage> StagePtr;

// Dynamic postprocessing library helper.
class PostProcessingLib
{
public:
	PostProcessingLib(const std::string &lib);
	PostProcessingLib(PostProcessingLib &&other);
	PostProcessingLib(const PostProcessingLib &other) = delete;
	PostProcessingLib &operator=(const PostProcessingLib &other) = delete;
	~PostProcessingLib();

	const void *GetSymbol(const std::string &symbol);

private:
	void *lib_ = nullptr;
	std::map<std::string, const void *> symbol_map_;
	std::mutex lock_;
};

class PostProcessor
{
public:
	PostProcessor(RPiCamApp *app);

	~PostProcessor();

	void LoadModules(const std::string &lib_dir);

	void Read(std::string const &filename);

	void SetCallback(PostProcessorCallback callback);

	void AdjustConfig(std::string const &use_case, StreamConfiguration *config);

	void Configure();

	void Start();

	void Process(CompletedRequestPtr &request);

	void Stop();

	void Teardown();

private:
	PostProcessingStage *createPostProcessingStage(char const *name);

	RPiCamApp *app_;
	std::vector<StagePtr> stages_;
	std::vector<PostProcessingLib> dynamic_stages_;
	void outputThread();

	std::queue<CompletedRequestPtr> requests_;
	std::queue<std::future<bool>> futures_;
	std::thread output_thread_;
	bool quit_;
	PostProcessorCallback callback_;
	std::mutex mutex_;
	std::condition_variable cv_;
};
