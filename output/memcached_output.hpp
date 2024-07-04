#pragma once

#include "output.hpp"
#include <libmemcached/memcached.hpp>
#include <hiredis/hiredis.h>

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
	redisContext *redis;
	redisReply *reply;
};
