/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * preview.cpp - preview window interface
 */

#include "core/options.hpp"

#include "preview.hpp"

Preview *make_null_preview(Options const *options);
Preview *make_egl_preview(Options const *options);
Preview *make_drm_preview(Options const *options);
Preview *make_qt_preview(Options const *options);

Preview *make_preview(Options const *options)
{
	if (options->nopreview)
		return make_null_preview(options);
#if QT_PRESENT
	else if (options->qt_preview)
	{
		Preview *p = make_qt_preview(options);
		if (p)
			std::cerr << "Made QT preview window" << std::endl;
		return p;
	}
#endif
	else
	{
		try
		{
#if LIBEGL_PRESENT
			Preview *p = make_egl_preview(options);
			if (p)
				std::cerr << "Made X/EGL preview window" << std::endl;
			return p;
#else
			throw std::runtime_error("egl libraries unavailable.");
#endif
		}
		catch (std::exception const &e)
		{
			try
			{
#if LIBDRM_PRESENT
				Preview *p = make_drm_preview(options);
				if (p)
					std::cerr << "Made DRM preview window" << std::endl;
				return p;
#else
				throw std::runtime_error("drm libraries unavailable.");
#endif
			}
			catch (std::exception const &e)
			{
				std::cerr << "Preview window unavailable" << std::endl;
				return make_null_preview(options);
			}
		}
	}

	return nullptr; // prevents compiler warning in debug builds
}
