/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Limited
 *
 * metadata.hpp - general metadata class
 */
#pragma once

// A simple class for carrying arbitrary metadata, for example about an image.

#include <any>
#include <map>
#include <mutex>
#include <string>

class Metadata
{
public:
	Metadata() = default;

	Metadata(Metadata const &other)
	{
		std::scoped_lock other_lock(other.mutex_);
		data_ = other.data_;
	}

	Metadata(Metadata &&other)
	{
		std::scoped_lock other_lock(other.mutex_);
		data_ = std::move(other.data_);
		other.data_.clear();
	}

	template <typename T>
	void Set(std::string const &tag, T &&value)
	{
		std::scoped_lock lock(mutex_);
		data_.insert_or_assign(tag, std::forward<T>(value));
	}

	template <typename T>
	int Get(std::string const &tag, T &value) const
	{
		std::scoped_lock lock(mutex_);
		auto it = data_.find(tag);
		if (it == data_.end())
			return -1;
		value = std::any_cast<T>(it->second);
		return 0;
	}

	void Clear()
	{
		std::scoped_lock lock(mutex_);
		data_.clear();
	}

	Metadata &operator=(Metadata const &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_ = other.data_;
		return *this;
	}

	Metadata &operator=(Metadata &&other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_ = std::move(other.data_);
		other.data_.clear();
		return *this;
	}

	void Merge(Metadata &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_.merge(other.data_);
	}

	template <typename T>
	T *GetLocked(std::string const &tag)
	{
		// This allows in-place access to the Metadata contents,
		// for which you should be holding the lock.
		auto it = data_.find(tag);
		if (it == data_.end())
			return nullptr;
		return std::any_cast<T>(&it->second);
	}

	template <typename T>
	void SetLocked(std::string const &tag, T &&value)
	{
		// Use this only if you're holding the lock yourself.
		data_.insert_or_assign(tag, std::forward<T>(value));
	}

	// Note: use of (lowercase) lock and unlock means you can create scoped
	// locks with the standard lock classes.
	// e.g. std::lock_guard<RPiController::Metadata> lock(metadata)
	void lock() { mutex_.lock(); }
	void unlock() { mutex_.unlock(); }

private:
	mutable std::mutex mutex_;
	std::map<std::string, std::any> data_;
};
