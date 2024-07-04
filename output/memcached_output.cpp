#include "memcached_output.hpp"

MemcachedOutput::MemcachedOutput(VideoOptions const *options) : Output(options)
{
	opt = options;

	const char *config_string = "--SOCKET=\"/var/run/memcached/memcached.sock\" --BINARY-PROTOCOL";
	memc = memcached(config_string, strlen(config_string));
	if (memc == NULL)
		LOG_ERROR("Error connecting to memcached");
	
	// Test memcache
	const char *key = "my_key";
    const char *value = "Hello, Memcached!";
    size_t key_len = strlen(key);
	size_t value_length = strlen(value);
	
	rc = memcached_set(memc, key, key_len, value, value_length, 0, 0);

	// Retrieve the value from Memcached
	size_t returned_value_length;
	uint32_t returned_flags;
	char *returned_value = memcached_get(memc, key, strlen(key),
											&returned_value_length, &returned_flags, &rc);

	if (rc == MEMCACHED_SUCCESS) {
		// Assert that the retrieved value matches the expected value
		assert(returned_value_length == value_length);
		assert(std::memcmp(returned_value, value, value_length) == 0);

		std::cout << "Retrieved value from Memcached matches expected value" << std::endl;
		std::cout << "Value: " << returned_value << std::endl;

		// Free the memory allocated by libmemcached
		free(returned_value);
	} else {
		LOG_ERROR("Failed to retrieve value from Memcached: ");
	}
	
	// Test redis
	redisContext *redis = redisConnect("127.0.0.1", 6379);
	if (redis == NULL || redis->err)
	{
		if (redis)
		{
			LOG_ERROR("Error redis");
			redisFree(redis);
		}
		else
		{
			LOG_ERROR("Can't allocate redis context");
		}
		exit(1);
	}

	redisReply *reply = (redisReply *)redisCommand(redis, "SET LibcameraService Alive");
	freeReplyObject(reply);
    redisFree(redis);
}

MemcachedOutput::~MemcachedOutput()
{	
	memcached_free(memc);
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
	memcached_return_t rc = memcached_set(memc, timestamp, strlen(timestamp), (char *)mem, size,(time_t)0, (uint32_t)16);
	if (rc == MEMCACHED_SUCCESS)
	{
		LOG(2, "Value added successfully to memcached: " << timestamp);
	}
	else
		LOG_ERROR("Error: " << rc << " adding value to memcached " << timestamp);
	
	// Redis connection
	redisContext *redis = redisConnect("127.0.0.1", 6379);
	if (redis == NULL || redis->err)
	{
		if (redis)
		{
			LOG_ERROR("Error redis");
			redisFree(redis);
		}
		else
		{
			LOG_ERROR("Can't allocate redis context");
		}
		return;
	}

	std::string time_str = timestamp;
	std::string redis_command = "XADD Libcamera * ";
	redis_command += "memcached " + time_str + " ";
	redis_command += "sensor_id Libcamera ";
	redis_command += "event NewFrame ";
	redis_command += "width " + std::to_string(opt->width) + " ";
	redis_command += "height " + std::to_string(opt->height) + " ";
	// redis_command += "framerate " + std::to_string(opt->framerate) + " ";
	// redis_command += "shutter " + std::to_string(opt->shutter) + " ";
	redis_command += "gain " + std::to_string(opt->gain) + " ";
	redis_command += "roi " + opt->roi;

	// Execute the Redis command
	redisReply *reply = (redisReply *)redisCommand(redis, redis_command.c_str());

    if (reply == NULL) {
		LOG_ERROR("Failed to execute command");
        redisFree(redis);
    }

    if (reply->type == REDIS_REPLY_ERROR) {
		LOG_ERROR("Redis reply error");
    } else {
		LOG(2, "Entry added to redis");
    }

    // Clean up
    freeReplyObject(reply);
    redisFree(redis);
}
