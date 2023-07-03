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
	{ libcamera::formats::R10,     10 },
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

int set_focus(const std::string &device, int focus_pos)
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

bool init_focus(std::string &device, int &minimum, int &maximum)
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
		minimum = ctrl.minimum;
		maximum = ctrl.maximum;
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
			("focus-steps", value<int32_t>(&focus_steps)->default_value(20),
			 "Step size for focus movements")
			("focus-min", value<int32_t>(&focus_min)->default_value(0),
			 "Lowest lens position for focus test")
			("focus-max", value<int32_t>(&focus_max)->default_value(65535),
			 "Highest lens position for focus test")
			("focus-wait", value<uint32_t>(&focus_wait)->default_value(3),
			 "Wait these many frames for the lens to complete movement (must be at least 1)")
			 ("focus-cycles", value<uint32_t>(&focus_cycles)->default_value(3),
			 "Number of focus cycles to run.")
			("dust-test", value<bool>(&dust_test)->default_value(false)->implicit_value(true),
			 "Runs the dust detection test")
			("low-threshold", value<uint32_t>(&lo_threshold)->default_value(0),
			 "Sets the low sample threshold (in 16-bits)")
			("high-threshold", value<uint32_t>(&hi_threshold)->default_value(0),
			 "Sets the high sample threshold (in 16-bits)")
			("hist", value<bool>(&hist)->default_value(false)->implicit_value(true),
			 "Switches on histogram display mode")
			("focus-fix", value<int32_t>(&focus_fix)->default_value(INT_MIN),
			 "Fixed focus position for cal/dust/quad tests")
			("quad-test", value<bool>(&quad_test)->default_value(false)->implicit_value(true),
			 "Analyse a 4-quadrant image")
			("black", value<uint32_t>(&black_level)->default_value(4096),
			 "Sets the black level for quad test (in 16-bits)")
			("temp", value<bool>(&check_temp)->default_value(false)->implicit_value(true),
			 "Report min and max sensor temperature")
			;
		// clang-format on
	}

	bool cal;
	bool focus_test;
	bool dust_test;
	bool quad_test;
	bool hist;
	bool check_temp;
	int32_t focus_steps;
	int32_t focus_min;
	int32_t focus_max;
	int32_t focus_fix;
	uint32_t focus_wait;
	uint32_t focus_cycles;
	uint32_t lo_threshold;
	uint32_t hi_threshold;
	uint32_t black_level;

	virtual bool Parse(int argc, char *argv[]) override
	{
		if (VideoOptions::Parse(argc, argv) == false)
			return false;

		/* Ensure AF is disabled when libcamera_prod drives the lens */
		if (focus_test || focus_fix != INT_MIN) {
			afMode_index = libcamera::controls::AfModeManual;
			lens_position = std::nullopt;
			set_default_lens_position = false;
		}

		return true;
	}
};

class LibcameraProd : public LibcameraApp
{
public:
	LibcameraProd()
		: LibcameraApp(std::make_unique<ProdOptions>()), focus_pos_(0), focus_min_pos_(0), focus_max_pos_(0),
		  focus_revdir_(false), focus_cycles_(0), lens_device_(), temp_min_(65536), temp_max_(-1)
	{
	}

	ProdOptions *GetOptions() const
	{
		return static_cast<ProdOptions *>(options_.get());
	}

	void run_calibration(CompletedRequestPtr req);
	bool run_focus_test(CompletedRequestPtr req, unsigned int count);
	void run_dust_test(CompletedRequestPtr req);
	void run_quad_test(CompletedRequestPtr req);
	void record_temp(int t);
	void report_temp();

	std::vector<uint16_t> linebuf_;
	std::vector<std::map<int, float>> foms_;
	int focus_pos_;
	int focus_min_pos_;
	int focus_max_pos_;
	bool focus_revdir_;
	unsigned int focus_cycles_;
	std::string lens_device_;
	int temp_min_, temp_max_;
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

	linebuf_.resize(info.width);
	for (unsigned int y = 0; y < info.height; y += 2)
	{
		uint16_t *s = &linebuf_[0];
		memcpy(s, (uint8_t const *)mem + y * info.stride, sizeof(uint16_t) * info.width);
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			uint16_t p = s[0];
			sum[0] += p;
			if (p < min[0]) min[0] = p;
			if (p > max[0]) max[0] = p;
			p = s[1];
			sum[1] += p;
			if (p < min[1]) min[1] = p;
			if (p > max[1]) max[1] = p;
		}

		s = &linebuf_[0];
		memcpy(s, (uint8_t const *)mem + (y + 1) * info.stride, sizeof(uint16_t) * info.width);
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			uint16_t p = s[0];
			sum[2] += p;
			if (p < min[2]) min[2] = p;
			if (p > max[2]) max[2] = p;
			p = s[1];
			sum[3] += p;
			if (p < min[3]) min[3] = p;
			if (p > max[3]) max[3] = p;
		}
	}

	for (unsigned int i = 0; i < 4; i++)
	{
		min[i] <<= shift;
		max[i] <<= shift;
		sum[i] *= (1 << shift);
	}

	std::stringstream s;
	s << "Min: [" << min[0] << ", " << min[1] << ", " << min[2] << ", " << min[3] << "] ";
	s << "Max: [" << max[0] << ", " << max[1] << ", " << max[2] << ", " << max[3] << "] ";
	s << "Mean: [" << sum[0] / (info.width * info.height / 4) << ", " << sum[1] / (info.width * info.height / 4) << ", "
	  << sum[2] / (info.width * info.height / 4) << ", " << sum[3] / (info.width * info.height / 4) << "]              ";
	std::cout << "\r" << s.str() << std::flush;
}

bool LibcameraProd::run_focus_test(CompletedRequestPtr req, unsigned int count)
{
	ProdOptions const *options = static_cast<ProdOptions *>(GetOptions());

	if ((count % options->focus_wait) == 0)
	{
		FrameInfo frame_info(req->metadata);

		if (focus_revdir_ ? (focus_pos_ >= focus_max_pos_) : (focus_pos_ <= focus_min_pos_))
			foms_.push_back({});

		foms_.back()[focus_pos_] = frame_info.focus;
		LOG(2, "Position: " << focus_pos_ << " Focus: " << frame_info.focus);

		if (focus_revdir_ ? (focus_pos_ <= focus_min_pos_) : (focus_pos_ >= focus_max_pos_))
		{
			if (focus_cycles_ == options->focus_cycles - 1)
			{
				std::stringstream s1, s2, s3, s4, s5;
				s1 << "Minimum : FoM [ ";
				s2 << " Pos [ ";
				s3 << "Maximum : FoM [ ";
				s4 << " Pos [ ";
				s5 << "Max/Min ratio : [ " << std::setprecision(5);
				
				for (auto const &foms : foms_)
				{
					auto [min, max] = std::minmax_element(foms.begin(), foms.end(),
										[](const auto &l, const auto &r) { return l.second < r.second; });
					s1 << min->second << " ";
					s2 << min->first << " ";
					s3 << max->second << " ";
					s4 << max->first << " ";
					s5 << max->second / min->second << " ";
				}

				s1 << "]"; s2 << "]"; s3 << "]"; s4 << "]"; s5 << "]";
				std::cout << s1.str() << s2.str() << std::endl << s3.str() << s4.str() << std::endl << s5.str() << std::endl;
				return true;
			}
			else
			{
				focus_revdir_ = !focus_revdir_;
				focus_cycles_++;
			}
		}
		else if (!focus_revdir_) {
			focus_pos_ = std::min(focus_max_pos_, focus_pos_ + options->focus_steps);
		}
		else {
			focus_pos_ = (focus_pos_ >= focus_max_pos_) ?
				focus_max_pos_ - 1 - (focus_max_pos_ - 1 - focus_min_pos_) % options->focus_steps :
				std::max(focus_min_pos_, focus_pos_ - options->focus_steps);
		}

		if (set_focus(lens_device_, focus_pos_))
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

	// Sum each (unshifted) component so we can compare their means
	double sum[4] = { 0.0, 0.0, 0.0, 0.0 };
	for (unsigned int y = 0; y < info.height; y += 2)
	{
		uint16_t const *s = (uint16_t const *)mem + y * info.stride / 2;
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			sum[0] += s[0];
			sum[1] += s[1];
		}

		s = (uint16_t const *)mem + (y + 1) * info.stride / 2;
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			sum[2] += s[0];
			sum[3] += s[1];
		}
	}
	double max_sum = std::max({ sum[0], sum[1], sum[2], sum[3] });

	// Normalise to largest component, shift to 16 bits, do counts
	double scale[4];
	for (unsigned i = 0; i < 4; i++)
		scale[i] = (max_sum * (1 << shift)) / sum[i];

	static constexpr unsigned int num_bins = 40;
	uint16_t bins[num_bins] = { 4000 };
	unsigned int counts[num_bins] = { 0 };
	for (unsigned int i = 1; i < num_bins; i++)
		bins[i] = bins[i - 1] + 1000;

	uint32_t lo = 0, hi = 0;
	for (unsigned int y = 0; y < info.height; y += 2)
	{
		uint16_t const *s = (uint16_t const *)mem + y * info.stride / 2;
		for (unsigned int x = 0; x < info.width; s += 2, x += 2)
		{
			// Note that normalisation could push high outliers above 65535
			uint32_t p[4] { (uint32_t)(*s * scale[0]), (uint32_t)(*(s + 1) * scale[1]),
							(uint32_t)(*(s + info.stride / 2) * scale[2]), (uint32_t)(*(s + info.stride / 2 + 1) * scale[3]) };

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

// Stuff for Quadrants test

static int find_max_step(uint16_t *buf, unsigned start, unsigned end, uint16_t &arg)
{
	unsigned a = 0;
	int max = 0;

	for(unsigned x = start - 2; x < end + 2; ++x)
	{
		int d = -buf[x-9] -2*buf[x-7] - 3*buf[x-5] - 3*buf[x-3] - 3*buf[x-1]
			+ 3*buf[x+1] + 3*buf[x+3] + 3*buf[x+5] + 2*buf[x+7] + buf[x+9];
		d = std::abs(d);
		if (d > max)
			max = d, a = x;
	}
	if (a < start || a >= end)
	{
		a = 0;
		max = 0;
	}
	arg = a;
	return max;
}

static bool fit_line(double params[2], uint16_t const *val, unsigned len)
{
	static const double MARGIN2      = 36.0; // max jitter +/- 6 pixels
	static const double MAX_GRADIENT = 0.26; // up to ~15 degrees off axis
	const unsigned l_3 = len/3; // Seed points must be in the middle 1/3
	double best_score = 0.0;

	params[0] = 0.0;
	params[1] = 0.0;

	// Fit a line to two points drawn from the middle third of the domain.
	// Use exhaustive search for one point and random sampling for the other.
	// Note that a value of 0 means that a point is undefined.
	for(unsigned x0 = l_3; x0 < 2*l_3; x0++)
	{
		if (!val[x0])
			continue;

		for(unsigned guess = 0; guess < 10; ++guess)
		{
			unsigned x1 = l_3 + (unsigned)rand() % l_3;
			if ((x1 < x0 + 64 && x0 < x1 + 64) || !val[x1])
				continue;

			double p1 = ((double)val[x1] - (double)val[x0]) /
				    ((double)x1 - (double)x0);
			double p0 = val[x0] - p1*x0;
			if (std::abs(p1) > MAX_GRADIENT)
				continue;

			double score = 0.0;
			for(unsigned x = 0; x < len; ++x)
			{
				double err = p0 + p1 * x - val[x];
				err *= err;
				if (val[x] && err < MARGIN2)
					score += MARGIN2 - err;
			}
			if (score > best_score)
			{
				best_score = score;
				params[0] = p0;
				params[1] = p1;
			}
		}
	}

	// As well as the two seed points there should be >= 2 more "good" points
	return (best_score >= 4.0 * MARGIN2);
}

static unsigned find_centile(uint32_t const *histogram, unsigned start, unsigned finish, unsigned centile)
{
	uint64_t tot = 0, cum = 0;
	unsigned i;

	for(i = start; i < finish; ++i)
		tot += histogram[i];
	tot *= centile;
	for(i = start; i < finish; ++i)
	{
		cum += histogram[i];
		if (100*cum >= tot)
			break;
	}
	return i;
}

void LibcameraProd::run_quad_test(CompletedRequestPtr req)
{
	StreamInfo info;
	libcamera::Stream *raw = RawStream(&info);
	unsigned black_level = static_cast<ProdOptions *>(GetOptions())->black_level;
	libcamera::FrameBuffer *buffer = req->buffers[raw];
	libcamera::Span span = Mmap(buffer)[0];
	void *mem = span.data();

	if (!buffer || !mem || info.width < 256 || info.height < 256)
		throw std::runtime_error("Invalid RAW buffer");

	auto it = bayer_formats.find(info.pixel_format);
	if (it == bayer_formats.end())
		throw std::runtime_error("Unsupported Bayer format");
	unsigned int shift = 16 - it->second;

	// Below, we'll use linebuf_[] to store either two rows of raw pixels,
	// or one row (of pixels) and one column (of indices) or vice versa.
	// We'll also compute the total light in each pair of rows and columns.
	linebuf_.resize(info.width + std::max(info.width, info.height));
	uint32_t *rowsums = new uint32_t [ (info.height + 1) / 2];
	uint32_t *colsums = new uint32_t [ (info.width + 1) / 2];

	// Find the largest H step (of any colour component) in the middle third of
	// each scanline. Meanwhile, count very bright pixels (after shifting to 16 bits)
	// and compute sums over pairs of rows. Subtract black level from each row sum.
	uint32_t bright_count = 0;
	for(unsigned y = 0; y < info.height; y++)
	{
		uint16_t *s = &linebuf_[info.height];
		memcpy(s, (uint16_t const *)mem + y * info.stride / 2, info.width*sizeof(uint16_t));
		uint32_t rowsum = 0;
		for(unsigned x = 0; x < info.width; x++)
		{
			uint16_t p = s[x] << shift;
			rowsum += p;
			if (p >= 0xFE00)
				bright_count++;
		}
		rowsum = (rowsum < info.width * black_level) ? 0 : (rowsum - info.width * black_level);
		if (!(y & 1))
			rowsums[y >> 1] = rowsum;
		else
			rowsums[y >> 1] += rowsum;
		find_max_step(s, info.width/3, 2*info.width/3, linebuf_[y]);
	}

	// Fail if more than 1% of pixels are likely to have saturated.
	printf("Saturation fraction: %4.2lf%%\n", 100.0 * bright_count / (double)(info.width * info.height));
	if (100 * bright_count > info.width * info.height)
		throw std::runtime_error("Image too bright for Quadrant test");

	// Try to find the vertical line.
	double vline_params[2], hline_params[2];
	if (!fit_line(vline_params, &linebuf_[0], info.height))
		throw std::runtime_error("Vertical line not detected");

	// Read columns into cached memory, and find the largest V step (of any colour
	// component) in the middle third of each column. Meanwhile, compute sums over
	// pairs of columns. Shift each column sum to 16 bits and subtract black level.
	for(unsigned x = 0; x < info.width; ++x)
	{
		uint16_t *s = &linebuf_[info.width];
		uint32_t colsum = 0;
		for(unsigned y = 0; y < info.height; ++y)
		{
			uint16_t p = *((uint16_t const *)mem + y * info.stride / 2 + x);
			s[y] = p;
			colsum += p;
		}
		colsum <<= shift;
		colsum = (colsum < info.height * black_level) ? 0 : (colsum - info.height * black_level);
		if (!(x & 1))
			colsums[x >> 1] = colsum;
		else
			colsums[x >> 1] += colsum;
		find_max_step(s, info.height/3, 2*info.height/3, linebuf_[x]);
	}

	// Try to find the horizontal line. Check the crossing angle is 90 +/- 5 degrees or so.
	if (!fit_line(hline_params, &linebuf_[0], info.width))
		throw std::runtime_error("Horizontal line not detected");
	if (std::abs(hline_params[1] + vline_params[1]) > 0.088)
		throw std::runtime_error("Detected lines not at right angles");

	// Find the intersection point. Both lines have to pass through the central
	// third of the width and of the height, but not necessarily to intersect there.
	// Let's require them to intersect in the central half of the width and height.
	unsigned intersect_x = (unsigned)(0.5 + (vline_params[0] + vline_params[1] * hline_params[0]) /
					  (1.0 - vline_params[1] * hline_params[1]));
	unsigned intersect_y = (unsigned)(0.5 + (hline_params[0] + hline_params[1] * vline_params[0]) /
					  (1.0 - vline_params[1] * hline_params[1]));
	printf("Lines intersect at %u, %u\n", intersect_x, intersect_y);
	if (4*intersect_x < info.width  || 4*intersect_x >= 3 * info.width ||
	    4*intersect_y < info.height || 4*intersect_y >= 3 * info.height)
	  throw std::runtime_error("Intersection point not central");

	// Find the bounding box containing 75% of the light in each of the 4 principal directions
	// from the intersection (with a 64 pixel margin, but no attempt to correct for rotation).
	// NB: bounding box coordinates are always even, so they align with pixel-quads.
	unsigned vbounds[2], hbounds[2];
	vbounds[0] = 2 * find_centile(rowsums, 0,                       (intersect_y - 64) >> 1, 25);
	vbounds[1] = 2 * find_centile(rowsums, (intersect_y + 65) >> 1, info.height >> 1,        75);
	hbounds[0] = 2 * find_centile(colsums, 0,                       (intersect_x - 64) >> 1, 25);
	hbounds[1] = 2 * find_centile(colsums, (intersect_x + 65) >> 1, info.width >> 1,         75);

	// Add up each colour component of pixel-quads in each image-quadrant within the bounding box,
	// ignoring pixel-quads within 64 pel of the boundary lines (this time, corrected for rotation)
	// and excluding quads containing very bright values that might have saturated.
	uint32_t count[4] = { 0, 0, 0, 0 };
	double mean[4][4];
	for(unsigned q = 0; q < 4; q++)
		for(unsigned c = 0; c < 4; c++)
			mean[q][c] = 0.0;
	for(unsigned y = vbounds[0]; y < vbounds[1] && y < info.height; y += 2)
	{
		uint16_t *s0 = &linebuf_[0];
		uint16_t *s1 = &linebuf_[info.width];
		memcpy(s0, (uint16_t const *)mem + y * info.stride / 2, info.width*sizeof(uint16_t));
		memcpy(s1, (uint16_t const *)mem + (y + 1) * info.stride / 2, info.width*sizeof(uint16_t));
		for(unsigned x = hbounds[0]; x < hbounds[1] && x < info.width; x += 2)
		{
			double hoffset = x + 0.5 - vline_params[0] - vline_params[1]*y;
			double voffset = y + 0.5 - hline_params[0] - hline_params[1]*x;
			if (std::abs(hoffset) >= 64.0 && std::abs(voffset) >= 64.0 &&
			    (std::max(std::max(s0[x], s0[x+1]), std::max(s1[x], s1[x+1])) << shift) < 0xFE00)
			{
				int q = ((voffset < 0.0) ? 0 : 2) + ((hoffset < 0.0) ? 0 : 1);
				mean[q][0] += s0[x];
				mean[q][1] += s0[x+1];
				mean[q][2] += s1[x];
				mean[q][3] += s1[x+1];
				count[q]++;
			}
		}
	}

	// Convert raw totals to means (shifted to 16 bits, with black level subtracted)
	// and print them (PLUS black level, for compatibility with cal/dust test output)
	for(unsigned q = 0; q < 4; q++)
	{
		double tot = 0.0;

		if (count[q] < 4096)
			throw std::runtime_error("Not enough pixels in quadrant");

		for(unsigned c = 0; c < 4; c++)
		{
			mean[q][c] = ((mean[q][c] * (1 << shift)) / (double)count[q]) - black_level;
			tot += mean[q][c];
		}

		printf("Means: [ %5d, %5d, %5d, %5d ] Fractions: [ %5.3lf, %5.3lf, %5.3lf, %5.3lf ]\n",
		 (int)(mean[q][0] + black_level),
		 (int)(mean[q][1] + black_level),
		 (int)(mean[q][2] + black_level),
		 (int)(mean[q][3] + black_level),
		 mean[q][0] / tot,
		 mean[q][1] / tot,
		 mean[q][2] / tot,
		 mean[q][3] / tot);
	}

	// Final image pass. Try to estimate the fraction of the image covered by the test pattern,
	// by counting pix-quads whose normalized correlation with the quadrant's mean colour is >= 1/8
	// with a bit of horizontal filtering to remove noise in the dark regions
	for(unsigned q = 0; q < 4; q++)
	{
		double mag2 = ((mean[q][0] * mean[q][0]) +
			       (mean[q][1] * mean[q][1]) +
			       (mean[q][2] * mean[q][2]) +
			       (mean[q][3] * mean[q][3]));
		mean[q][0] /= mag2;
		mean[q][1] /= mag2;
		mean[q][2] /= mag2;
		mean[q][3] /= mag2;
	}
	bright_count = 0;
	for(unsigned y = 0; y < info.height; y += 2)
	{
		uint16_t *s0 = &linebuf_[0];
		uint16_t *s1 = &linebuf_[info.width];
		memcpy(s0, (uint16_t const *)mem + y * info.stride / 2, info.width*sizeof(uint16_t));
		memcpy(s1, (uint16_t const *)mem + (y + 1) * info.stride / 2, info.width*sizeof(uint16_t));
		bool prev = true;
		for(unsigned x = 0; x < info.width; x += 2)
		{
			double hoffset = x + 0.5 - vline_params[0] - vline_params[1]*y;
			double voffset = y + 0.5 - hline_params[0] - hline_params[1]*x;
			int q = ((voffset < 0.0) ? 0 : 2) + ((hoffset < 0.0) ? 0 : 1);
			double corr =
				((s0[x]   << shift) - (double)black_level) * mean[q][0] +
				((s0[x+1] << shift) - (double)black_level) * mean[q][1] +
				((s1[x]   << shift) - (double)black_level) * mean[q][2] +
				((s1[x+1] << shift) - (double)black_level) * mean[q][3];
			if (corr >= 0.125 && prev)
				bright_count++;
			prev = (corr >= 0.125);
		}
	}
	printf("Area factor: %4.1lf%%\n", 400.0 * bright_count / (double)(info.width * info.height));

#if 0
	// XXX debug visualization only
	FILE * fpdebug = fopen("tmp.ppm", "w");
	fprintf(fpdebug, "P6 %d %d 255\n", info.width/2, info.height/2);
	for(unsigned y = 0; y < info.height; y += 2)
	{
		uint16_t *s0 = &linebuf_[0];
		uint16_t *s1 = &linebuf_[info.width];
		memcpy(s0, (uint16_t const *)mem + y * info.stride / 2, info.width*sizeof(uint16_t));
		memcpy(s1, (uint16_t const *)mem + (y + 1) * info.stride / 2, info.width*sizeof(uint16_t));
		bool prev = true;
		for(unsigned x = 0; x < info.width; x += 2)
		{
			double hoffset = x + 0.5 - vline_params[0] - vline_params[1]*y;
			double voffset = y + 0.5 - hline_params[0] - hline_params[1]*x;
			int q = ((voffset < 0.0) ? 0 : 2) + ((hoffset < 0.0) ? 0 : 1);
			int r = std::sqrt(s1[x+1] << shift);
			int g = std::sqrt(s0[x+1] << shift);
			int b = std::sqrt(s0[x] << shift);
			if ((info.height-2-y) == ((colsums[x>>1]>>17)&~1)) b = 255;
			if (x == ((rowsums[y>>1]>>17)&~1)) r = 255;
			if (y >= vbounds[0] && y < vbounds[1] && x >= hbounds[0] && x < hbounds[1] &&
			    std::abs(hoffset) >= 64.0 && std::abs(voffset) >= 64.0 &&
			    (std::max(std::max(s0[x], s0[x+1]), std::max(s1[x], s1[x+1])) << shift) < 0xFE00)
			{
				r >>= 1;
				g >>= 1;
				b >>= 1;
			} else {
				double corr =
				  ((s0[x]   << shift) - black_level) * mean[q][0] +
				  ((s0[x+1] << shift) - black_level) * mean[q][1] +
				  ((s1[x]   << shift) - black_level) * mean[q][2] +
				  ((s1[x+1] << shift) - black_level) * mean[q][3];
				if (corr < 0.125 || !prev)
					g = (g+255)>>1;
				prev = (corr >= 0.125);
			}
			if (std::abs(x - vline_params[0] - vline_params[1]*y) <= 1.0 ||
			    std::abs(y - hline_params[0] - hline_params[1]*x) <= 1.0)
			  g = 255;
			fputc(r, fpdebug);
			fputc(g, fpdebug);
			fputc(b, fpdebug);
		}
	}
	fclose(fpdebug);
#endif

	delete [] colsums;
	delete [] rowsums;
}

void LibcameraProd::record_temp(int t)
{
	if (t < temp_min_)
		temp_min_ = t;
	if (t > temp_max_)
		temp_max_ = t;
}

void LibcameraProd::report_temp()
{
	std::cout << "Temperature: min " << temp_min_ << " max " << temp_max_ << " diff " << (temp_max_ - temp_min_)
			  << std::endl;
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

	if (options->focus_test || options->focus_fix != INT_MIN)
	{
		if (!(init_focus(app.lens_device_, app.focus_min_pos_, app.focus_max_pos_)))
			throw std::runtime_error("Could not initialise focus driver!");
		app.focus_min_pos_ = std::max(app.focus_min_pos_, options->focus_min);
		app.focus_max_pos_ = std::min(app.focus_max_pos_, options->focus_max);
		app.focus_pos_ = (options->focus_test) ? app.focus_min_pos_ : options->focus_fix;
		set_focus(app.lens_device_, app.focus_pos_);
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

		if (options->check_temp)
		{
			auto tctl = req->metadata.get(libcamera::controls::SensorTemperature);
			if (tctl)
				app.record_temp(*tctl);
		}

		app.ShowPreview(req, app.VideoStream());
	}

	if (options->check_temp)
		app.report_temp();

	std::cout << std::endl;

	if (options->dust_test && req)
		app.run_dust_test(req);

	else if (options->quad_test && req)
		app.run_quad_test(req);

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

			if (options->quad_test)
			{
				options->mode_string = "1920:1080:12:U";
				options->mode = Mode(options->mode_string);
				options->focus_test = false;
				options->dust_test = false;
			}
			else if (options->dust_test || options->cal)
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
