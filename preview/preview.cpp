/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * preview.cpp - preview window interface
 */

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

	if (options->Get().nopreview)
		return factory.CreatePreview("null")(options);
	else if (options->Get().qt_preview)
	{
		if (factory.HasPreview("qt"))
		{
			LOG(1, "Made QT preview window");
			return factory.CreatePreview("qt")(options);
		}
	}
	else
	{
		try
		{
			if (factory.HasPreview("egl"))
			{
				LOG(1, "Made X/EGL preview window");
				return factory.CreatePreview("egl")(options);
			}
			throw std::runtime_error("egl libraries unavailable.");
		}
		catch (std::exception const &e)
		{
			try
			{
				if (factory.HasPreview("drm"))
				{
					LOG(1, "Made DRM preview window");
					return factory.CreatePreview("drm")(options);
				}
				throw std::runtime_error("drm libraries unavailable.");
			}
			catch (std::exception const &e)
			{
				LOG(1, "Preview window unavailable");
				return factory.CreatePreview("null")(options);
			}
		}
	}

	return nullptr; // prevents compiler warning in debug builds
}
