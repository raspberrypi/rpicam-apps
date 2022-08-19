/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */
#include "memcached_output.hpp"
#include <chrono>
#include <iostream>
#include <libmemcached/memcached.hpp>
#include <sw/redis++/redis++.h>
using namespace sw::redis;
using namespace std;

MemcachedOutput::MemcachedOutput(VideoOptions const *options) : Output(options)
{
	// Connect
	opt = options;

	const char *config_string = "--SOCKET=\"/var/run/memcached/memcached.sock\" --BINARY-PROTOCOL";
	memc = memcached(config_string, strlen(config_string));
	if (memc == NULL)
		cerr << "Error connecting to memcached" << endl;
	auto redis = Redis("tcp://127.0.0.1:6379");
	redis.set("key", "val");
	auto val = redis.get("key"); // val is of type OptionalString. See 'API Reference' section for details.
	if (val)
	{
		// Dereference val to get the returned value of std::string type.
		std::cout << *val << std::endl;
	} // else key doesn't exist.
}

MemcachedOutput::~MemcachedOutput()
{
	memcached_free(memc);
}

void int64ToChar(char a[], int64_t n)
{
	memcpy(a, &n, 8);
}

void MemcachedOutput::outputBuffer(void *mem, size_t size, int64_t /*timestamp_us*/ J, uint32_t /*flags*/)
{
	int64_t t =
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
			.count();
	char timestamp[16];
	sprintf(timestamp, "%li", t);
	// Flag set to 16 since the python bmemcached protocol library recognizes binary data with flag 16
	// This way the bmemcached library does not decode when reading.
	memcached_return_t rc = memcached_set(memc, timestamp, strlen(timestamp), (char *)mem, opt->width * opt->height,
										  (time_t)0, (uint32_t)16);
	if (rc == MEMCACHED_SUCCESS)
	{
		LOG(2, "Value added successfully to memcached: " << timestamp);
	}
	else
		LOG(2, "Error: " << rc << " adding value to memcached " << timestamp);
	auto redis = Redis("tcp://127.0.0.1:6379");
	using Attrs = std::vector<std::pair<std::string, std::string>>;
	std::string time_str = timestamp;
	Attrs attrs = {
		{ "memcached", time_str },
		{ "sensor_id", "Libcamera" },
		{ "event", "NewFrame" },
		{ "width", std::to_string(opt->width) },
		{ "height", std::to_string(opt->height) },
		{ "framerate", std::to_string(opt->framerate) },
		{ "shutter", std::to_string(opt->shutter) },
		{ "gain", std::to_string(opt->gain) },
		{ "roi", opt->roi },
	};
	auto id = redis.xadd("Libcamera", "*", attrs.begin(), attrs.end());
}
