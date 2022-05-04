/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd.
 *
 * libcamera_prod.cpp - libcamera prod test app.
 */

#include <fcntl.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include <chrono>
#include <string>

#include "core/libcamera_encoder.hpp"
#include "encoder/null_encoder.hpp"
#include "output/output.hpp"

#include "core/frame_info.hpp"

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

static int xioctl(int fd, unsigned long ctl, void *arg)
{
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

int set_focus(const std::string &device, unsigned int focus_pos)
{
	v4l2_control ctrl = {};
	int ret = -1;
	int fd = -1;

	fd = open(device.c_str(), O_RDWR, 0);
	if (fd < 0)
	{
		std::cerr << "Open lens device error" << std::endl;
		goto err;
	}

	ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
	ctrl.value = focus_pos;
	ret = xioctl(fd, VIDIOC_S_CTRL, &ctrl);

err:
	close(fd);
	return ret;
}

bool init_focus(std::string &device, unsigned int &focus_max_pos)
{
	for (int i = 0; i < 4; i++)
	{
		std::string dev("/dev/v4l-subdev" + std::to_string(i));
		int fd = -1;

		fd = open(dev.c_str(), O_RDWR, 0);
		if (fd < 0)
			continue;

		v4l2_queryctrl ctrl;
		ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
		if (xioctl(fd, VIDIOC_QUERYCTRL, &ctrl))
		{
			close(fd);
			continue;
		}

		close(fd);
		device = dev;
		focus_max_pos = ctrl.maximum;
		set_focus(device, 0);

		return true;
	}

	return false;
}

struct ProdOptions : public VideoOptions
{
	ProdOptions() : VideoOptions()
	{
		using namespace boost::program_options;
		// clang-format off
		options_.add_options()
			("focus-test", value<bool>(&focus_test)->default_value(false)->implicit_value(true),
			 "Runs a focus test")
			("focus-steps", value<uint32_t>(&focus_steps)->default_value(20),
			 "Step size for focus movements")
			("focus-wait", value<uint32_t>(&focus_wait)->default_value(3),
			 "Wait these many frames for the lens to complete movement (must be at least 1)")
			("dust-test", value<bool>(&dust_test)->default_value(false)->implicit_value(true),
			 "Runs the dust detection test")
			("low-threshold", value<uint32_t>(&lo_threshold)->default_value(0),
			 "Sets the low sample threshold (in 16-bits)")
			("high-threshold", value<uint32_t>(&hi_threshold)->default_value(0),
			 "Sets the high sample threshold (in 16-bits)")
			;
		// clang-format on
	}

	bool focus_test;
	bool dust_test;
	uint32_t focus_steps;
	uint32_t focus_wait;
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
	std::map<unsigned int, float> foms;

	app.OpenCamera();
	app.ConfigureVideo(LibcameraProd::FLAG_VIDEO_RAW);
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	unsigned int focus_pos = 0, focus_max_pos = 0;
	std::string lens_device;
	if (options->focus_test)
	{
		if (!(init_focus(lens_device, focus_max_pos)))
			throw std::runtime_error("Could not initialise focus driver!");
	}

	for (unsigned int count = 0;; count++)
	{
		LibcameraProd::Msg msg = app.Wait();
		CompletedRequestPtr req = std::get<CompletedRequestPtr>(msg.payload);

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

		if (options->dust_test)
		{
			StreamInfo info;
			libcamera::Stream *raw = app.RawStream(&info);

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

		if (options->focus_test)
		{
			if ((count % options->focus_wait) == 0)
			{
				FrameInfo frame_info(req->metadata);

				foms[focus_pos] = frame_info.focus;
				if (options->verbose)
					std::cerr << "Position: " << focus_pos << " Focus: " << frame_info.focus << std::endl;

				if (focus_pos == focus_max_pos)
				{
					auto [min, max] = std::minmax_element(foms.begin(), foms.end(),
												[](const auto &l, const auto &r) { return l.second < r.second; });

					std::cout << "Maximum focus " << max->second << " at position " << max->first << std::endl;
					std::cout << "Minimum focus " << min->second << " at position " << min->first << std::endl;
					std::cout << "Max/Min ratio " << std::setprecision(5) << max->second / min->second << std::endl;
					break;
				}

				focus_pos = std::min<unsigned int>(focus_pos + options->focus_steps, focus_max_pos);
				if (set_focus(lens_device, focus_pos))
					throw std::runtime_error("cannot set focus!");
			}
		}
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

			if (options->dust_test)
			{
				options->mode_string = "10000:10000:12:U";
				options->mode = Mode(options->mode_string);
				options->focus_test = false;
			}

			if (options->focus_test)
			{
				options->frames = 0;
				options->timeout = 0;

				if (!options->shutter || !options->gain)
					throw std::runtime_error("Must set fixed shutter and gain for the focus test!");
			}

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
