/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * segmentation.hpp - segmentation result
 */

#pragma once

#include <string>
#include <vector>

struct Segmentation
{
	Segmentation(int w, int h, std::vector<std::string> l, const std::vector<uint8_t> &s)
		: width(w), height(h), labels(l), segmentation(s)
	{
	}
	int width;
	int height;
	std::vector<std::string> labels;
	std::vector<uint8_t> segmentation;
};
