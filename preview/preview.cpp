/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * preview.cpp - preview window interface
 */

#include <cstdlib>
#include <filesystem>

#include "core/dl_lib.hpp"
#include "core/options.hpp"

#include "config.h"
#include "preview.hpp"

namespace fs = std::filesystem;

PreviewFactory &PreviewFactory::GetInstance()
{
	static PreviewFactory instance;
	return instance;
}

void PreviewFactory::RegisterPreview(const std::string &name, PreviewCreateFunc create_func)
{
	previews_[name] = create_func;
}

PreviewCreateFunc PreviewFactory::CreatePreview(const std::string &name)
{
	auto it = previews_.find(name);
	if (it != previews_.end())
		return it->second;
	return nullptr;
}

bool PreviewFactory::HasPreview(const std::string &name) const
{
	return previews_.find(name) != previews_.end();
}

void PreviewFactory::LoadPreviewLibraries(const std::string &lib_dir)
{
	const fs::path path(!lib_dir.empty() ? lib_dir : PREVIEW_LIB_DIR);
	const std::string ext(".so");

	if (!fs::exists(path))
		return;

	// Dynamically load all .so files from the system preview lib path.
	// This will automatically register the stages with the factory.
	for (auto const &p : fs::recursive_directory_iterator(path))
	{
		if (p.path().extension() == ext)
		{
			const std::string library_path = p.path().string();

			// Check if this library has already been loaded
			if (loaded_library_paths_.find(library_path) == loaded_library_paths_.end())
			{
				preview_libraries_.emplace_back(library_path);
				loaded_library_paths_.insert(library_path);
			}
		}
	}
}

RegisterPreview::RegisterPreview(char const *name, PreviewCreateFunc create_func)
{
	PreviewFactory::GetInstance().RegisterPreview(name, create_func);
}

Preview *make_preview(Options const *options)
{
	auto &factory = PreviewFactory::GetInstance();
	factory.LoadPreviewLibraries(options->Get().preview_libs);

	if (!options->Get().nopreview)
	{
		std::vector<std::string> previews;
		if (!options->Get().preview_backend.empty())
		{
			// The user has forced a specific backend; try only that one.
			previews = { options->Get().preview_backend };
		}
		else
		{
			previews = { "egl", "drm" };
			// On a native Wayland session, prefer the native Wayland EGL preview
			// to avoid the XWayland round-trip that the X11 EGL preview incurs. On
			// X11 this environment variable is unset, so nothing changes. An empty
			// value is treated as unset.
			char const *wayland_display = getenv("WAYLAND_DISPLAY");
			if (wayland_display && *wayland_display)
				previews.insert(previews.begin(), "wayland-egl");
			if (options->Get().qt_preview)
				previews.insert(previews.begin(), "qt");
		}

		for (auto const &p : previews)
		{
			try
			{
				if (factory.HasPreview(p))
				{
					Preview *r = factory.CreatePreview(p)(options);
					LOG(1, "Made " + p + " preview window");
					return r;
				}
			}
			catch (std::exception const &e)
			{
				LOG(1, "Failed to create " + p + " preview");
			}
		}
		LOG(1, "Preview window unavailable");
	}

	return factory.CreatePreview("null")(options); // this really shouldn't fail
}
