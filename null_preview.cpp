/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * null_preview.cpp - dummy "show nothing" preview window.
 */

#include "null_preview.hpp"

Preview *make_null_preview(Options const *options)
{
	return new NullPreview(options);
}
