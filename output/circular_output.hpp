/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * circular_output.hpp - Write output to a circular buffer.
 */

#pragma once

#include "output.hpp"

// A simple circular buffer implementation used by the CircularOutput class.

class CircularBuffer
{
public:
	CircularBuffer(size_t size) : size_(size), buf_(size), rptr_(0), wptr_(0) {}
	bool Empty() const { return rptr_ == wptr_; }
	size_t Available() const { return wptr_ == rptr_ ? size_ - 1 : (size_ - wptr_ + rptr_) % size_ - 1; }
	void Skip(unsigned int n) { rptr_ = (rptr_ + n) % size_; }
	// The dst function allows bytes read to go straight to memory or a file etc.
	void Read(std::function<void(void *src, unsigned int n)> dst, unsigned int n)
	{
		if (rptr_ + n >= size_)
		{
			dst(&buf_[rptr_], size_ - rptr_);
			n -= size_ - rptr_;
			rptr_ = 0;
		}
		dst(&buf_[rptr_], n);
		rptr_ += n;
	}
	void Pad(unsigned int n) { wptr_ = (wptr_ + n) % size_; }
	void Write(const void *ptr, unsigned int n)
	{
		if (wptr_ + n >= size_)
		{
			memcpy(&buf_[wptr_], ptr, size_ - wptr_);
			n -= size_ - wptr_;
			ptr = static_cast<const uint8_t *>(ptr) + size_ - wptr_;
			wptr_ = 0;
		}
		memcpy(&buf_[wptr_], ptr, n);
		wptr_ += n;
	}

private:
	const size_t size_;
	std::vector<uint8_t> buf_;
	size_t rptr_, wptr_;
};

// Write frames to a circular buffer, and dump them to disk when we quit.

class CircularOutput : public Output
{
public:
	CircularOutput(VideoOptions const *options);
	~CircularOutput();

protected:
	void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;
	void timestampReady(int64_t timestamp) override;

private:
	CircularBuffer cb_;
	FILE *fp_;
};
