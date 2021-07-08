/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.hpp - Write output to file.
 */

#pragma once

#include "output.hpp"

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
	FILE *fp_;
	unsigned int count_;
	int64_t file_start_time_ms_;
};
