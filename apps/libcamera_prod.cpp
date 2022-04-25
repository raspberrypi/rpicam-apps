/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd.
 *
 * libcamera_prod.cpp - libcamera prod test app.
 */

#include <chrono>

#include "core/libcamera_encoder.hpp"
#include "encoder/null_encoder.hpp"
#include "output/output.hpp"

using namespace std::placeholders;

static const std::map<libcamera::PixelFormat, unsigned int> bayer_formats =
{
	{ libcamera::formats::SRGGB10, 10 },
	{ libcamera::formats::SGRBG10, 10 },
	{ libcamera::formats::SBGGR10, 10 },
	{ libcamera::formats::SGBRG10, 10 },
	{ libcamera::formats::SRGGB12, 12 },
	{ libcamera::formats::SGRBG12, 12 },
	{ libcamera::formats::SBGGR12, 12 },
	{ libcamera::formats::SGBRG12, 12 },
};

struct ProdOptions : public VideoOptions
{
	ProdOptions() : VideoOptions()
	{
		using namespace boost::program_options;
		// clang-format off
		options_.add_options()
			("low-threshold", value<uint32_t>(&lo_threshold)->default_value(0),
			 "Sets the low sample threshold (in 16-bits)")
			("high-threshold", value<uint32_t>(&hi_threshold)->default_value(0),
			 "Sets the high sample threshold (in 16-bits)")
			;
		// clang-format on
	}

	uint32_t lo_threshold;
	uint32_t hi_threshold;

	virtual bool Parse(int argc, char *argv[]) override
	{
		if (VideoOptions::Parse(argc, argv) == false)
			return false;

		return true;
	}
};

class LibcameraProd : public LibcameraApp
{
public:
	LibcameraProd()
		: LibcameraApp(std::make_unique<ProdOptions>())
	{
	}

	ProdOptions *GetOptions() const
	{
		return static_cast<ProdOptions *>(options_.get());
	}
};

// The main even loop for the application.

static void event_loop(LibcameraProd &app)
{
	ProdOptions const *options = static_cast <ProdOptions *>(app.GetOptions());
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create((VideoOptions *) options));

	app.OpenCamera();
	app.ConfigureVideo(LibcameraProd::FLAG_VIDEO_RAW);
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0;; count++)
	{
		LibcameraProd::Msg msg = app.Wait();

		if (msg.type != LibcameraProd::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		if (count == 0)
		{
			libcamera::StreamConfiguration const &cfg = app.RawStream()->configuration();
			std::cerr << "Raw stream: " << cfg.size.width << "x" << cfg.size.height << " stride " << cfg.stride
					  << " format " << cfg.pixelFormat.toString() << std::endl;
		}

		if (options->verbose)
			std::cerr << "Viewfinder frame " << count << std::endl;
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->frames && options->timeout &&
					   (now - start_time > std::chrono::milliseconds(options->timeout));
		bool frameout = options->frames && count >= options->frames;
		if (timeout || frameout)
		{
			app.StopCamera();
			return;
		}

		StreamInfo info;
		libcamera::Stream *raw = app.RawStream(&info);

		CompletedRequestPtr req = std::get<CompletedRequestPtr>(msg.payload);
		libcamera::FrameBuffer *buffer = req->buffers[raw];
		libcamera::Span span = app.Mmap(buffer)[0];
		void *mem = span.data();

		if (!buffer || !mem)
			throw std::runtime_error("Invalid RAW buffer");

		auto it = bayer_formats.find(info.pixel_format);
		if (it == bayer_formats.end())
			throw std::runtime_error("Unsupported Bayer format");
		unsigned int shift = 16 - it->second;

		uint32_t lo = 0, hi = 0;
		for (unsigned int y = 0; y < info.height; y++)
		{
			uint16_t *s = (uint16_t *)mem + y * info.stride / 2;
			for (unsigned int x = 0; x < info.width; s++, x++)
			{
				unsigned int p = *s << shift;
				if (p > options->lo_threshold)
					lo++;
				if (p > options->hi_threshold)
					hi++;
			}
		}

		std::cout << info.width * info.height << " total samples" << std::endl;
		std::cout << lo << " samples above threshold of " << options->lo_threshold << std::endl;
		std::cout << hi << " samples above threshold of " << options->hi_threshold << std::endl;
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraProd app;
		ProdOptions *options = static_cast<ProdOptions *>(app.GetOptions());
		if (options->Parse(argc, argv))
		{
			options->denoise = "cdn_off";
			options->nopreview = true;
			options->mode_string = "10000:10000:12:U";
			options->mode = Mode(options->mode_string);
			if (options->verbose)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
		return -1;
	}
	return 0;
}
