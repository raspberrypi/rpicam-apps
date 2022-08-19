/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * net_output.hpp - send output over network.
 */
#pragma once

#include "output.hpp"
#include <libmemcached/memcached.hpp>
#include <sw/redis++/redis++.h>

using namespace std;
using namespace sw::redis;

class MemcachedOutput : public Output
{
public:
	MemcachedOutput(VideoOptions const *options);
	~MemcachedOutput();

protected:
	void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;

private:
	memcached_st *memc;
	const VideoOptions *opt;
	memcached_return_t rc;
	memcached_return error;
};
