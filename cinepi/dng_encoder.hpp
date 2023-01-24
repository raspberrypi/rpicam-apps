#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <deque>
#include <thread>
#include <boost/circular_buffer.hpp>

#include "encoder/encoder.hpp"
#include "raw_options.hpp"

class DngEncoder : public Encoder
{
public:
	DngEncoder(RawOptions const *options);
	~DngEncoder();
	// Encode the given buffer.
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us) override;
	// void Encode(CompletedRequestPtr &completed_request);
	void EncodeBuffer2(int fd, size_t size, void *mem, StreamInfo const &info, size_t losize, void *lomem, StreamInfo const &loinfo, int64_t timestamp_us, CompletedRequest::ControlList const &metadata);
	void resetFrameCount(){
		index_ = 0;
	}
	int recording(){
		return cache_buffer_.size();
	}
	uint64_t getFrameCount(){
		return frames_;
	}

    CompletedRequest::ControlList const *metadata_;

private:
	// How many threads to use. Whichever thread is idle will pick up the next frame.
	static const int NUM_ENC_THREADS = 6;

	// These threads do the actual encoding.
	void encodeThread(int num);

	void cacheThread(int num);
	// Handle the output buffers in another thread so as not to block the encoders. The
	// application can take its time, after which we return this buffer to the encoder for
	// re-use.
	void outputThread();

	bool encodeCheck_;
	bool abortEncode_;
	bool abortOutput_;
	bool resetCount_;
	uint64_t index_;
	uint64_t frames_;
	uint64_t frameStop_;

    RawOptions const *options_;

	void dng_save(uint8_t const *mem, StreamInfo const &info, uint8_t const *lomem, StreamInfo const &loinfo, size_t losize,
			libcamera::ControlList const &metadata, std::string const &filename, std::string const &cam_name,
			RawOptions const *options, uint64_t fn);

	struct EncodeItem
	{
		void *mem;
        size_t size;
		StreamInfo info;
		void *lomem;
		size_t losize;
		StreamInfo loinfo;
		CompletedRequest::ControlList met;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<EncodeItem> encode_queue_;
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];

	struct CachedItem
	{
		void *mem;
        size_t size;
		StreamInfo info;
		void *lomem;
		size_t losize;
		StreamInfo loinfo;
		CompletedRequest::ControlList met;
		int64_t timestamp_us;
		uint64_t index;
	};
	boost::circular_buffer<CachedItem> cache_buffer_;
	std::queue<CachedItem> cache_queue_;
	std::mutex cache_mutex_;
	std::condition_variable cache_cond_var_;
	std::thread cache_thread_[NUM_ENC_THREADS];
};