/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */
#include "memcached_output.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <libmemcached/memcached.hpp>
#include <sstream>
using namespace std;

MemcachedOutput::MemcachedOutput(VideoOptions const *options) : Output(options)
{
	// Connect
	const char *config_string = "--SERVER=haku --BINARY-PROTOCOL";
	memc = memcached(config_string, strlen(config_string));
	if (memc == NULL)
	{
		cout << "Error connecting to memcached\n";
	}
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
	char timestamp[64];
	sprintf(timestamp, "%ld", t);
	// Flag set to 16 since the python bmemcached protocol library recognizes binary data with flag 16
	// This way the bmemcached library does not decode when reading.
	memcached_return_t rc =
		memcached_set(memc, timestamp, strlen(timestamp), (char *)mem, strlen((char *)mem), (time_t)0, (uint32_t)16);
	if (rc == MEMCACHED_SUCCESS)
		cout << "Value added successfully to memcached: " << timestamp << "\n";
	else
		cout << "Error: " << rc << " adding value to memcached " << timestamp << "\n";
}
