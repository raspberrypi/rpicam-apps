/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * preview.hpp - preview window interface
 */

#pragma once

#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <libcamera/base/span.h>

#include "core/stream_info.hpp"

struct Options;
class DlLib;

class Preview
{
public:
	typedef std::function<void(int fd)> DoneCallback;

	Preview(Options const *options) : options_(options) {}
	virtual ~Preview() {}
	// This is where the application sets the callback it gets whenever the viewfinder
	// is no longer displaying the buffer and it can be safely recycled.
	void SetDoneCallback(DoneCallback callback) { done_callback_ = callback; }
	virtual void SetInfoText(const std::string &text) {}
	// Display the buffer. You get given the fd back in the BufferDoneCallback
	// once its available for re-use.
	virtual void Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info) = 0;
	// Reset the preview window, clearing the current buffers and being ready to
	// show new ones.
	virtual void Reset() = 0;
	// Check if preview window has been shut down.
	virtual bool Quit() { return false; }
	// Return the maximum image size allowed.
	virtual void MaxImageSize(unsigned int &w, unsigned int &h) const = 0;

protected:
	DoneCallback done_callback_;
	Options const *options_;
};

typedef Preview *(*PreviewCreateFunc)(Options const *options);

class PreviewFactory
{
public:
	static PreviewFactory &GetInstance();

	// Prevent copying and assignment
	PreviewFactory(const PreviewFactory &) = delete;
	PreviewFactory &operator=(const PreviewFactory &) = delete;

	void RegisterPreview(const std::string &name, PreviewCreateFunc create_func);
	void LoadPreviewLibraries(const std::string &lib_dir);

	PreviewCreateFunc CreatePreview(const std::string &name);

	bool HasPreview(const std::string &name) const;
	const std::map<std::string, PreviewCreateFunc> &GetPreviews() const { return previews_; }

private:
	PreviewFactory() = default;
	~PreviewFactory() = default;

	std::map<std::string, PreviewCreateFunc> previews_;
	std::vector<DlLib> preview_libraries_;
	std::set<std::string> loaded_library_paths_;
};

struct RegisterPreview
{
	RegisterPreview(char const *name, PreviewCreateFunc create_func);
};

Preview *make_preview(Options const *options);
