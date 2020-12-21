/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * null_preview.hpp - dummy "show nothing" preview window.
 */

#pragma once

#include <iostream>

#include "preview.hpp"

class NullPreview : public Preview
{
public:
	NullPreview(Options const &options) : Preview(options) {
		if (options.verbose)
			std::cout << "Running without preview window" << std::endl;
	}
	~NullPreview() {}
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, size_t size, int width, int height, int stride) override
	{
		done_callback_(fd);
	}
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	void Reset() override
	{
	}
private:
};
