/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * null_preview.cpp - dummy "show nothing" preview window.
 */

#include <iostream>

#include "core/options.hpp"

#include "preview.hpp"

class NullPreview : public Preview
{
public:
	NullPreview(Options const *options) : Preview(options)
	{
		if (options->verbose)
			std::cerr << "Running without preview window" << std::endl;
	}
	~NullPreview() {}
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info) override
	{
		done_callback_(fd);
	}
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	void Reset() override {}
	// Return the maximum image size allowed. Zeroes mean "no limit".
	virtual void MaxImageSize(unsigned int &w, unsigned int &h) const override { w = h = 0; }

private:
};

Preview *make_null_preview(Options const *options)
{
	return new NullPreview(options);
}
