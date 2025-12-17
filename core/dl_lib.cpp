/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * dl_lib.cpp - Dynamic loading library
 */

#include "core/dl_lib.hpp"
#include "core/logging.hpp"

DlLib::DlLib(const std::string &lib, int flags)
{
	if (!lib.empty())
	{
		lib_ = dlopen(lib.c_str(), flags);
		if (!lib_)
			LOG_ERROR("Unable to open " << lib << " with error: " << dlerror());
	}
}

DlLib::DlLib(DlLib &&other)
{
	lib_ = other.lib_;
	symbol_map_ = std::move(other.symbol_map_);
	other.lib_ = nullptr;
}

DlLib::~DlLib()
{
	if (lib_)
		dlclose(lib_);
}

const void *DlLib::GetSymbol(const std::string &symbol)
{
	if (!lib_)
		return nullptr;

	std::scoped_lock<std::mutex> l(lock_);

	const auto it = symbol_map_.find(symbol);
	if (it == symbol_map_.end())
	{
		const void *fn = dlsym(lib_, symbol.c_str());

		if (!fn)
		{
			LOG_ERROR("Unable to find postprocessing symbol " << symbol << " with error: " << dlerror());
			return nullptr;
		}

		symbol_map_[symbol] = fn;
	}

	return symbol_map_[symbol];
}
