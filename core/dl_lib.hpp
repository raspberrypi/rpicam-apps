/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * dl_lib.cpp - Dynamic loading library
 */

#pragma once

#include <dlfcn.h>

#include <map>
#include <memory>
#include <mutex>

// Dynamic library helper.
class DlLib
{
public:
	DlLib(const std::string &lib, int flags = RTLD_LAZY);
	DlLib(DlLib &&other);
	DlLib(const DlLib &other) = delete;
	DlLib &operator=(const DlLib &other) = delete;
	~DlLib();

	const void *GetSymbol(const std::string &symbol);

private:
	void *lib_ = nullptr;
	std::map<std::string, const void *> symbol_map_;
	std::mutex lock_;
};
