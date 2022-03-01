/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */
#include "memcached_output.hpp"
#include <iostream>
#include <libmemcached/memcached.hpp>
using namespace std;

MemcachedOutput::MemcachedOutput(VideoOptions const *options) : Output(options)
{
	// Connect
	const char *config_string = "--SERVER=localhost";
	memcached_st *memc = memcached(config_string, strlen(config_string));
	if (memc == NULL)
	{
		cout << "Error connecting to memcached\n";
	}

	// Add value
	const char *key = "mykey";
	const char *value = "myvalue";
	memcached_return_t rc = memcached_set(memc, key, strlen(key), value, strlen(value), (time_t)0, (uint32_t)0);
	if (rc == MEMCACHED_SUCCESS)
		cout << "Value added successfully\n";
	else
		cout << "Error adding value\n";

	// Retrieve value
	memcached_return error;
	uint32_t flags;
	size_t return_value_length;
	const char *response = memcached_get(memc, key, strlen(key), &return_value_length, &flags, &error);
	cout << response;
}

MemcachedOutput::~MemcachedOutput()
{
	//memcached_free(memc);
}

void MemcachedOutput::outputBuffer(void *mem, size_t size, int64_t /*timestamp_us*/, uint32_t /*flags*/)
{
	const char *key = "mykey";
	const char *value = "myvalue";
	cout << "Inside outputBuffer\n";
	if (memc == NULL)
	{
		cout << "memc is NULL in outputBuffer\n";
	}
	memcached_set(memc, key, strlen(key), value, strlen(value), (time_t)0, (uint32_t)0);
	cout << "End outputBuffer\n";
}
