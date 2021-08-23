/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * preview.cpp - preview window interface
 */

#include "preview.hpp"

Preview *make_null_preview(Options const *options);
Preview *make_egl_preview(Options const *options);
Preview *make_drm_preview(Options const *options);

Preview *make_preview(Options const *options)
{
	if (options->nopreview)
		return make_null_preview(options);
	else
	{
		try
		{
#if LIBEGL_PRESENT
			return make_egl_preview(options);
			if (options->verbose)
				std::cout << "Made X/EGL preview window" << std::endl;
#else
			throw std::runtime_error("egl libraries unavailable.");
#endif
		}
		catch (std::exception const &e)
		{
			try
			{
#if LIBDRM_PRESENT
				return make_drm_preview(options);
				if (options->verbose)
					std::cout << "Made DRM preview window" << std::endl;
#else
				throw std::runtime_error("drm libraries unavailable.");
#endif
			}
			catch (std::exception const &e)
			{
				std::cout << "Preview window unavailable" << std::endl;
				return make_null_preview(options);
			}
		}
	}
}
