/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_detect.hpp - object detector result
 */

#pragma once

#include <sstream>

#include <libcamera/geometry.h>

struct Detection
{
	Detection(int c, const std::string &n, float conf, int x, int y, int w, int h)
		: category(c), name(n), confidence(conf), box(x, y, w, h)
	{
	}
	int category;
	std::string name;
	float confidence;
	libcamera::Rectangle box;
	std::string toString() const
	{
		std::stringstream output;
		output.precision(2);
		output << name << "[" << category << "] (" << confidence << ") @ " << box.x << "," << box.y << " " << box.width
			   << "x" << box.height;
		return output.str();
	}
};
