/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.hpp - Write output to file.
 */

#pragma once

#include <filesystem>

#include "file_name_manager.hpp"
#include "output.hpp"

namespace fs = std::filesystem;

class FileOutput : public Output
{
public:
	FileOutput(VideoOptions const *options);
	~FileOutput();

protected:
	void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;

private:
	void openFile(int64_t timestamp_us);
	void closeFile();
	void saveDng(void *mem);
	void saveFile(void *mem, size_t size, int64_t timestamp_us, uint32_t flags);
	FILE *fp_;
	int64_t file_start_time_ms_;
	FileNameManager fileNameManager_;
};
