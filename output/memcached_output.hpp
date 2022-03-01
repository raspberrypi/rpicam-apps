/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * net_output.hpp - send output over network.
 */
#pragma once

#include <libmemcached/memcached.hpp>

#include "output.hpp"
using namespace std;

class MemcachedOutput : public Output
{
public:
	MemcachedOutput(VideoOptions const *options);
	~MemcachedOutput();

protected:
	void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;

private:
	memcached_server_st *servers = NULL;
	memcached_st *memc;
	memcached_return_t rc;
};
