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
#include "core/logging.hpp"

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
		LOG_ERROR("Open lens device error");
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
			("cal", value<bool>(&cal)->default_value(false)->implicit_value(true),
			 "Switches on calibration mode")
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
			("hist", value<bool>(&hist)->default_value(false)->implicit_value(true),
			 "Switches on calibration mode")
			;
		// clang-format on
	}

	bool cal;
	bool focus_test;
	bool dust_test;
	bool hist;
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

	void run_calibration(CompletedRequestPtr req);
	bool run_focus_test(CompletedRequestPtr req, unsigned int count);
	void run_dust_test(CompletedRequestPtr req);

	std::map<unsigned int, float> foms_;
	unsigned int focus_max_pos_;
	std::string lens_device_;
};

void LibcameraProd::run_calibration(CompletedRequestPtr req)
{
	StreamInfo info;
	libcamera::Stream *raw = RawStream(&info);

	libcamera::FrameBuffer *buffer = req->buffers[raw];
	libcamera::Span span = Mmap(buffer)[0];
	void *mem = span.data();

	if (!buffer || !mem)
		throw std::runtime_error("Invalid RAW buffer");

	auto it = bayer_formats.find(info.pixel_format);
	if (it == bayer_formats.end())
		throw std::runtime_error("Unsupported Bayer format");
	unsigned int shift = 16 - it->second;

	double sum[4] = { 0.0, 0.0, 0.0, 0.0 };
	uint16_t min[4] = { 65535, 65535, 65535, 65535 };
	uint16_t max[4] = { 0, 0, 0, 0 };

	for (unsigned int y = 0; y < info.height; y += 2)
	{
		uint16_t *s = (uint16_t *)mem + y * info.stride / 2;
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			uint16_t p[4] { (uint16_t)(*s << shift), (uint16_t)(*(s + 1) << shift),
							(uint16_t)(*(s + info.stride / 2) << shift), (uint16_t)(*(s + info.stride / 2 + 1) << shift) };

			for (unsigned int i = 0; i < 4; i++)
			{
				sum[i] += p[i];
				if (p[i] < min[i]) min[i] = p[i];
				if (p[i] > max[i]) max[i] = p[i];
			}
		}
	}

	for (unsigned int i = 0; i < 4; i++)
		std::cout << "Channel " << i << " : min " << min[i] << " max " << max[i]
				  << " mean " << sum[i] / (info.width * info.height / 4) << std::endl;
}

bool LibcameraProd::run_focus_test(CompletedRequestPtr req, unsigned int count)
{
	ProdOptions const *options = static_cast<ProdOptions *>(GetOptions());
	unsigned int focus_pos = 0;

	if ((count % options->focus_wait) == 0)
	{
		FrameInfo frame_info(req->metadata);

		foms_[focus_pos] = frame_info.focus;
		LOG(2, "Position: " << focus_pos << " Focus: " << frame_info.focus);

		if (focus_pos == focus_max_pos_)
		{
			auto [min, max] = std::minmax_element(foms_.begin(), foms_.end(),
										[](const auto &l, const auto &r) { return l.second < r.second; });

			std::cout << "Maximum focus " << max->second << " at position " << max->first << std::endl;
			std::cout << "Minimum focus " << min->second << " at position " << min->first << std::endl;
			std::cout << "Max/Min ratio " << std::setprecision(5) << max->second / min->second << std::endl;
			return true;
		}

		focus_pos = std::min<unsigned int>(focus_pos + options->focus_steps, focus_max_pos_);
		if (set_focus(lens_device_, focus_pos))
			throw std::runtime_error("cannot set focus!");
	}

	return false;
}

void LibcameraProd::run_dust_test(CompletedRequestPtr req)
{
	ProdOptions const *options = static_cast<ProdOptions *>(GetOptions());
	StreamInfo info;
	libcamera::Stream *raw = RawStream(&info);

	libcamera::FrameBuffer *buffer = req->buffers[raw];
	libcamera::Span span = Mmap(buffer)[0];
	void *mem = span.data();

	if (!buffer || !mem)
		throw std::runtime_error("Invalid RAW buffer");

	auto it = bayer_formats.find(info.pixel_format);
	if (it == bayer_formats.end())
		throw std::runtime_error("Unsupported Bayer format");
	unsigned int shift = 16 - it->second;

	// Shift and calculate mean
	double sum[4] = { 0.0, 0.0, 0.0, 0.0 };
	for (unsigned int y = 0; y < info.height; y += 2)
	{
		uint16_t *s = (uint16_t *)mem + y * info.stride / 2;
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			*s = *s << shift;
			*(s + 1) = *(s + 1) << shift;
			*(s + info.stride / 2) = *(s + info.stride / 2) << shift;
			*(s + info.stride / 2 + 1) = *(s + info.stride / 2 + 1) << shift;

			uint16_t p[4] { (uint16_t)(*s), (uint16_t)(*(s + 1)),
							(uint16_t)(*(s + info.stride / 2)), (uint16_t)(*(s + info.stride / 2 + 1)) };

			for (unsigned int i = 0; i < 4; i++)
				sum[i] += p[i];
		}
	}

	// Normalise means and do counts
	double mean[4] = { sum[0] / (info.width * info.height / 4), sum[1] / (info.width * info.height / 4),
						sum[2] / (info.width * info.height / 4), sum[3] / (info.width * info.height / 4) };
	double max_mean = std::max({ mean[0], mean[1], mean[2], mean[3] });

	static constexpr unsigned int num_bins = 40;
	uint16_t bins[num_bins] = { 4000 };
	unsigned int counts[num_bins] = { 0 };
	for (unsigned int i = 1; i < num_bins; i++)
		bins[i] = bins[i - 1] + 1000;

	uint32_t lo = 0, hi = 0;
	for (unsigned int y = 0; y < info.height; y += 2)
	{
		uint16_t *s = (uint16_t *)mem + y * info.stride / 2;
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			uint16_t p[4] { (uint16_t)(*s * max_mean / mean[0]), (uint16_t)(*(s + 1) * max_mean / mean[1]),
							(uint16_t)(*(s + info.stride / 2) * max_mean / mean[2]), (uint16_t)(*(s + info.stride / 2 + 1) * max_mean / mean[3]) };

			for (unsigned int i = 0; i < 4; i++)
			{
				if (p[i] <= options->lo_threshold)
					lo++;
				if (p[i] <= options->hi_threshold)
					hi++;
				if (options->hist)
				{
					for (unsigned int j = 0; j < num_bins; j++)
						if (p[i] <= bins[j])
							counts[j]++;
				}
			}
		}
	}

	std::cout << info.width * info.height << " total samples" << std::endl;
	std::cout << lo << " samples under threshold of " << options->lo_threshold << std::endl;
	std::cout << hi << " samples under threshold of " << options->hi_threshold << std::endl;
	if (options->hist)
	{
		std::cout << "Histogram:" << std::endl;
		for (unsigned int j = 0; j < num_bins; j++)
			std::cout << "    [" << bins[j] << "] " << counts[j] << std::endl;
	}
}

// The main even loop for the application.

static void event_loop(LibcameraProd &app)
{
	ProdOptions const *options = static_cast<ProdOptions *>(app.GetOptions());
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create((VideoOptions *) options));
	CompletedRequestPtr req;

	app.OpenCamera();
	app.ConfigureVideo(LibcameraProd::FLAG_VIDEO_RAW);
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	if (options->focus_test)
	{
		if (!(init_focus(app.lens_device_, app.focus_max_pos_)))
			throw std::runtime_error("Could not initialise focus driver!");
	}

	for (unsigned int count = 0;; count++)
	{
		LibcameraProd::Msg msg = app.Wait();

		req = std::get<CompletedRequestPtr>(msg.payload);
		if (msg.type == LibcameraEncoder::MsgType::Quit)
			break;
		if (msg.type != LibcameraProd::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		if (count == 0)
		{
			libcamera::StreamConfiguration const &cfg = app.RawStream()->configuration();
			LOG(1, "Raw stream: " << cfg.size.width << "x" << cfg.size.height << " stride " << cfg.stride
					<< " format " << cfg.pixelFormat.toString());
		}

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->frames && options->timeout &&
					   (now - start_time > std::chrono::milliseconds(options->timeout));
		bool frameout = options->frames && count >= options->frames;
		if (timeout || frameout)
			break;

		if (options->cal)
			app.run_calibration(req);

		if (options->focus_test)
			if (app.run_focus_test(req, count))
				break;

		app.ShowPreview(req, app.VideoStream());
	}

	if (options->dust_test && req)
		app.run_dust_test(req);

	app.StopCamera();
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

			if (options->dust_test || options->cal)
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

			if (options->verbose > 1)
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
