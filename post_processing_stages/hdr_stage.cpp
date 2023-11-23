/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * hdr_stage.cpp - HDR and DRC processing
 */

// This post-processing stage can do DRC (Dynamic Range Compression) and HDR-like effects.
// The difference between the two is really only in the parameter configuration where DRC
// would probably take a single image that is exposed "about right" whereas this particular
// HDR method will want several under-exposed images.

// For a DRC capture, use something like:
// rpicam-still -o test.jpg --post-process-file drc.json

// HDR will accumulate multiple frames faster without colour denoise, so maybe:
// rpicam-still -o test.jpg --ev -2 --denoise cdn_off --post-process-file hdr.json

// Obviously this runs as a post-processing stage on fully-processed 8-bit images. Normally
// I'd rather do HDR in the raw domain where the signals are still linear, and there are
// more bits to play with, but clearly that's not possible here. It does mean some of the
// pixel manipulations, especially when it comes to colour, are a bit random. You have
// been warned. Enjoy!

#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"
#include "core/stream_info.hpp"

#include "image/image.hpp"

#include "post_processing_stages/histogram.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include "post_processing_stages/pwl.hpp"

using Stream = libcamera::Stream;

struct LpFilterConfig
{
	double strength; // smaller value actually smoothes more
	Pwl threshold; // defines the level of pixel differences that will be smoothed over
};

// A TonemapPoint gives a target value within the full dynamic range where we would like
// the given quantile (actually, inter-quantile mean) in the image's histogram to go.
// Additionally there are limits to how much the current value can be scaled up or down.

struct TonemapPoint
{
	double q; // quantile
	double width; // width of inter-quantile mean there
	double target; // where in the dynamic range to target it
	double max_up; // maximum increase to current value (gain >= 1)
	double max_down; // maximum decrease to current value (gain <= 1)
	void Read(boost::property_tree::ptree const &params)
	{
		q = params.get<double>("q");
		width = params.get<double>("width");
		target = params.get<double>("target");
		max_up = params.get<double>("max_up");
		max_down = params.get<double>("max_down");
	}
};

struct GlobalTonemapConfig
{
	std::vector<TonemapPoint> points;
	double strength; // 1.0 follows the target tonemap, 0.0 ignores it
};

struct LocalTonemapConfig
{
	Pwl pos_strength; // gain applied to local contrast when brighter than neighbourhood
	Pwl neg_strength; // gain applied to local contrast when darker than neighbourhood
	double colour_scale; // allows colour saturation to be increased or reduced slightly
};

struct HdrConfig
{
	unsigned int num_frames; // number of frames to accumulate
	LpFilterConfig lp_filter; // low pass filter settings
	GlobalTonemapConfig global_tonemap; // global tonemap settings
	LocalTonemapConfig local_tonemap; // settings for adding back local contrast
	std::string jpeg_filename; // set this if you want individual jpegs saved as well
};

struct HdrImage
{
	HdrImage() : width(0), height(0), dynamic_range(0) {}
	HdrImage(int w, int h, int num_pixels) : width(w), height(h), pixels(num_pixels), dynamic_range(0) {}
	int width;
	int height;
	std::vector<int16_t> pixels;
	int dynamic_range; // 1 more than the maximum pixel value
	int16_t &P(unsigned int offset) { return pixels[offset]; }
	int16_t P(unsigned int offset) const { return pixels[offset]; }
	void Clear() { std::fill(pixels.begin(), pixels.end(), 0); }
	void Accumulate(uint8_t const *src, int stride);
	HdrImage LpFilter(LpFilterConfig const &config) const;
	Pwl CreateTonemap(GlobalTonemapConfig const &config) const;
	void Tonemap(HdrImage const &lp, HdrConfig const &config);
	void Extract(uint8_t *dest, int stride) const;
	Histogram CalculateHistogram() const;
	void Scale(double factor);
};

static void add_Y_pixels(int16_t *dest, uint8_t const *src, int width, int stride, int height)
{
	for (int y = 0; y < height; y++, src += stride)
	{
		for (int x = 0; x < width; x++)
			*(dest++) += src[x];
	}
}

// Add the new image buffer to this "accumulator" image. We just add them as
// we don't have the horsepower to do any fancy alignment or anything.
// Actually, spreading it across a few threads doesn't seem to help much, though
// compiling with "gcc -mfpu=neon-fp-armv8 -ftree-vectorize" gives a big
// improvement.

void HdrImage::Accumulate(uint8_t const *src, int stride)
{
	int16_t *dest = &P(0);
	int width2 = width / 2, stride2 = stride / 2;
	std::thread thread1(add_Y_pixels, dest, src, width, stride, height);

	dest += width * height;
	src += stride * height;

	// U and V components
	for (int y = 0; y < height; y++, src += stride2)
	{
		for (int x = 0; x < width2; x++)
			*(dest++) += src[x] - 128;
	}

	dynamic_range += 256;

	thread1.join();
}

// Forward pass of the IIR low pass filter.

static void forward_pass(std::vector<double> &fwd_pixels, std::vector<double> &fwd_weight_sums, HdrImage const &in,
						 std::vector<double> &weights, std::vector<double> &threshold, int width, int height, int size,
						 double strength)

{
	// (Should probably initialise the top/left elements of fwd_pixels/fwd_weight_sums...)
	for (int y = size; y < height; y++)
	{
		unsigned int off = y * width + size;
		for (int x = size; x < width; x++, off++)
		{
			int pixel = in.P(off);
			double scale = 10 / threshold[pixel];
			double pixel_wt_sum = pixel * strength, wt_sum = strength;

			// Compiler generates faster code from this:
			unsigned int p[4], idx[4];
			double wt[4];
			p[0] = fwd_pixels[off - width - 1];
			p[1] = fwd_pixels[off - width];
			p[2] = fwd_pixels[off - width + 1];
			p[3] = fwd_pixels[off - 1];
			idx[0] = std::abs(static_cast<int>(p[0]) - pixel) * scale;
			idx[1] = std::abs(static_cast<int>(p[1]) - pixel) * scale;
			idx[2] = std::abs(static_cast<int>(p[2]) - pixel) * scale;
			idx[3] = std::abs(static_cast<int>(p[3]) - pixel) * scale;
			wt[0] = idx[0] >= weights.size() ? 0.0 : weights[idx[0]];
			wt[1] = idx[1] >= weights.size() ? 0.0 : weights[idx[1]];
			wt[2] = idx[2] >= weights.size() ? 0.0 : weights[idx[2]];
			wt[3] = idx[3] >= weights.size() ? 0.0 : weights[idx[3]];
			pixel_wt_sum += wt[0] * p[0] + wt[1] * p[1] + wt[2] * p[2] + wt[3] * p[3];
			wt_sum += wt[0] + wt[1] + wt[2] + wt[3];

			fwd_pixels[off] = pixel_wt_sum / wt_sum;
			fwd_weight_sums[off] = wt_sum;
		}
	}
}

// Low pass IIR filter. We perform a forwards and a reverse pass, finally combining
// the results to get a smoothed but vaguely edge-preserving version of the
// accumulator image. You could imagine implementing alternative (more sophisticated)
// filters.

HdrImage HdrImage::LpFilter(LpFilterConfig const &config) const
{
	// Cache threshold values, computing them would be slow.
	std::vector<double> threshold = config.threshold.GenerateLut<double>();

	// Cache values of e^(-x^2) for 0 <= x <= 3, it will be much quicker
	std::vector<double> weights(31);
	for (int d = 0; d <= 30; d++)
		weights[d] = exp(-d * d / 100.0);

	int size = 1;
	double strength = config.strength;

	// Forward pass.
	std::vector<double> fwd_weight_sums(width * height);
	std::vector<double> fwd_pixels(width * height);

	HdrImage out(width, height, width * height);
	out.dynamic_range = dynamic_range;

	// Run the forward pass in other thread, so that the two passes run in parallel.
	std::thread fwd_pass(forward_pass, std::ref(fwd_pixels), std::ref(fwd_weight_sums), std::ref(*this),
						 std::ref(weights), std::ref(threshold), width, height, size, strength);

	// Reverse pass, but otherwise the same as the forward pass. There could be a small
	// saving in omitting it, but it's not huge given that they run in parallel.
	std::vector<double> rev_weight_sums(width * height);
	std::vector<double> rev_pixels(width * height);
	// (Should probably initialise the bottom/right elements of rev_pixels/rev_weight_sums...)
	for (int y = height - 1 - size; y >= 0; y--)
	{
		unsigned int off = y * width + width - 1 - size;
		for (int x = width - 1 - size; x >= 0; x--, off--)
		{
			int pixel = P(off);
			double scale = 10 / threshold[pixel];
			double pixel_wt_sum = pixel * strength, wt_sum = strength;

			// Compiler generates faster code from this:
			unsigned int p[4], idx[4];
			double wt[4];
			p[0] = rev_pixels[off + width + 1];
			p[1] = rev_pixels[off + width];
			p[2] = rev_pixels[off + width - 1];
			p[3] = rev_pixels[off + 1];
			idx[0] = std::abs(static_cast<int>(p[0]) - pixel) * scale;
			idx[1] = std::abs(static_cast<int>(p[1]) - pixel) * scale;
			idx[2] = std::abs(static_cast<int>(p[2]) - pixel) * scale;
			idx[3] = std::abs(static_cast<int>(p[3]) - pixel) * scale;
			wt[0] = idx[0] >= weights.size() ? 0.0 : weights[idx[0]];
			wt[1] = idx[1] >= weights.size() ? 0.0 : weights[idx[1]];
			wt[2] = idx[2] >= weights.size() ? 0.0 : weights[idx[2]];
			wt[3] = idx[3] >= weights.size() ? 0.0 : weights[idx[3]];
			pixel_wt_sum += wt[0] * p[0] + wt[1] * p[1] + wt[2] * p[2] + wt[3] * p[3];
			wt_sum += wt[0] + wt[1] + wt[2] + wt[3];

			rev_pixels[off] = pixel_wt_sum / wt_sum;
			rev_weight_sums[off] = wt_sum;
		}
	}

	fwd_pass.join();

	// Combine.
	unsigned int off = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++, off++)
			out.P(off) = (fwd_pixels[off] * fwd_weight_sums[off] + rev_pixels[off] * rev_weight_sums[off]) /
						 (fwd_weight_sums[off] + rev_weight_sums[off]);
	}

	return out;
}

Histogram HdrImage::CalculateHistogram() const
{
	std::vector<uint32_t> bins(dynamic_range);
	std::fill(bins.begin(), bins.end(), 0);
	for (int i = 0; i < width * height; i++)
		bins[P(i)]++;
	return Histogram(&bins[0], dynamic_range);
}

// This creates the tone curve that we apply to the low pass image using the list of
// quantiles and targets in the configuration.

Pwl HdrImage::CreateTonemap(GlobalTonemapConfig const &config) const
{
	int maxval = dynamic_range - 1;
	Histogram histogram = CalculateHistogram();

	Pwl tonemap;
	tonemap.Append(0, 0);
	for (auto &tp : config.points)
	{
		double iqm = histogram.InterQuantileMean(tp.q - tp.width, tp.q + tp.width);
		double target = tp.target * 4096;
		target = std::clamp(target, iqm * tp.max_down, iqm * tp.max_up);
		target = std::clamp<double>(target, 0, 4095);
		target = iqm + (target - iqm) * config.strength;
		tonemap.Append(iqm, target);
	}
	tonemap.Append(maxval, maxval);

	return tonemap;
}

// Tonemap the low pass image according to the global tone curve, and add back the high pass
// detail (given by the original pixel minus the low pass equivalent).

void HdrImage::Tonemap(HdrImage const &lp, HdrConfig const &config)
{
	Pwl tonemap = CreateTonemap(config.global_tonemap);

	// Make LUTs for the all the Pwls, it'll be much quicker.
	std::vector<int> tonemap_lut = tonemap.GenerateLut<int>();
	std::vector<double> pos_strength_lut = config.local_tonemap.pos_strength.GenerateLut<double>();
	std::vector<double> neg_strength_lut = config.local_tonemap.neg_strength.GenerateLut<double>();
	double colour_scale = config.local_tonemap.colour_scale;

	int maxval = dynamic_range - 1;
	for (int y = 0; y < height; y++)
	{
		unsigned int off_Y = y * width;
		unsigned int off_U = y * width / 4 + width * height;
		unsigned int off_V = off_U + width * height / 4;
		for (int x = 0; x < width; x++, off_Y++)
		{
			int Y_lp_orig = lp.P(off_Y), Y_hp = P(off_Y) - Y_lp_orig;
			int Y_lp_mapped = tonemap_lut[Y_lp_orig];
			double strength = (Y_hp > 0 ? pos_strength_lut : neg_strength_lut)[Y_lp_orig];
			int Y_final = std::clamp(Y_lp_mapped + (int)(strength * Y_hp), 0, maxval);
			P(off_Y) = Y_final;
			if (!(x & 1) && !(y & 1))
			{
				double f = (Y_final + 1) / (double)(Y_lp_orig + 1);
				// The values here are non-linear to colours can come out slightly saturated.
				// The colour_scale allows us to tweak that a little if we want.
				f = (f - 1) * colour_scale + 1;
				int U = P(off_U), V = P(off_V);
				P(off_U) = U * f;
				P(off_V) = V * f;
				off_U++, off_V++;
			}
		}
	}
}

// Write image back out to 8-bit buffer with given stride.

void HdrImage::Extract(uint8_t *dest, int stride) const
{
	double ratio = dynamic_range / 256;
	const int16_t *Y_ptr = &pixels[0];
	const int16_t *U_ptr = Y_ptr + width * height, *V_ptr = U_ptr + width * height / 4;
	uint8_t *dest_y = dest;
	uint8_t *dest_u = dest_y + stride * height, *dest_v = dest_u + stride * height / 4;

	for (int y = 0; y < height; y++, dest_y += stride)
	{
		for (int x = 0; x < width; x++)
			dest_y[x] = *(Y_ptr++) / ratio;
	}

	int w = width / 2, h = height / 2, s = stride / 2;
	for (int y = 0; y < h; y++, dest_u += s, dest_v += s)
	{
		for (int x = 0; x < w; x++)
		{
			int U = *(U_ptr++) / ratio;
			int V = *(V_ptr++) / ratio;
			dest_u[x] = std::clamp(U + 128, 0, 255);
			dest_v[x] = std::clamp(V + 128, 0, 255);
		}
	}
}

// Apply simple scaling to all pixels.

void HdrImage::Scale(double factor)
{
	for (unsigned int i = 0; i < pixels.size(); i++)
		pixels[i] *= factor;
	dynamic_range *= factor;
}

class HdrStage : public PostProcessingStage
{
public:
	HdrStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void AdjustConfig(std::string const &use_case, StreamConfiguration *config) override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	StreamInfo info_;
	HdrConfig config_;
	unsigned int frame_num_;
	std::mutex mutex_;
	HdrImage acc_, lp_;
};

#define NAME "hdr"

char const *HdrStage::Name() const
{
	return NAME;
}

void HdrStage::Read(boost::property_tree::ptree const &params)
{
	config_.num_frames = params.get<unsigned int>("num_frames");

	config_.lp_filter.strength = params.get<double>("lp_filter_strength");
	config_.lp_filter.threshold.Read(params.get_child("lp_filter_threshold"));

	for (auto &p : params.get_child("global_tonemap_points"))
	{
		TonemapPoint tp;
		tp.Read(p.second);
		config_.global_tonemap.points.push_back(tp);
	}
	config_.global_tonemap.strength = params.get<double>("global_tonemap_strength");

	Pwl pos_strength, neg_strength;
	pos_strength.Read(params.get_child("local_pos_strength"));
	neg_strength.Read(params.get_child("local_neg_strength"));
	double strength = params.get<double>("local_tonemap_strength");
	config_.local_tonemap.colour_scale = params.get<double>("local_colour_scale");

	// A strength of 1 should give the value in the function; a strength of 0 should give the value 1.
	pos_strength.Map([this, strength](double x, double y) {
		y = y * strength + 1 - strength;
		config_.local_tonemap.pos_strength.Append(x, y);
	});
	neg_strength.Map([this, strength](double x, double y) {
		y = y * strength + 1 - strength;
		config_.local_tonemap.neg_strength.Append(x, y);
	});

	config_.jpeg_filename = params.get<std::string>("jpeg_filename", "");
}

void HdrStage::AdjustConfig(std::string const &use_case, StreamConfiguration *config)
{
	// HDR will want to capture several full res frames as fast as possible
	// for which we need several buffers in the queue.
	if (use_case == "still" && config->bufferCount < 3)
		config->bufferCount = 3;
}

void HdrStage::Configure()
{
	stream_ = app_->StillStream(&info_);
	if (!stream_)
		return;
	if (stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("HdrStage: only supports YUV420");

	// Allocate and initialise the big accumulator image.
	frame_num_ = 0;
	acc_ = HdrImage(info_.width, info_.height, info_.width * info_.height * 3 / 2);
	acc_.Clear();
	lp_ = HdrImage(info_.width, info_.height, info_.width * info_.height);
}

bool HdrStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false; // in viewfinder mode, do nothing

	std::lock_guard<std::mutex> lock(mutex_);

	// Once the HDR frame has been done it's not clear what to do... so let's just
	// send the subsequent frames through unmodified.
	if (frame_num_ >= config_.num_frames)
		return false;

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	std::vector<libcamera::Span<uint8_t>> const &buffers = w.Get();
	libcamera::Span<uint8_t> buffer = buffers[0];
	uint8_t *image = buffer.data();

	// Accumulate frame.
	LOG(1, "Accumulating frame " << frame_num_);
	acc_.Accumulate(image, info_.stride);

	// Optionally save individual JPEGs of each of the constituent images. Obviously this
	// will rather slow down the accumulation process.
	if (!config_.jpeg_filename.empty())
	{
		char filename[128];
		snprintf(filename, sizeof(filename), config_.jpeg_filename.c_str(), frame_num_);
		filename[sizeof(filename) - 1] = 0;
		StillOptions const *options = dynamic_cast<StillOptions *>(app_->GetOptions());
		if (options)
			jpeg_save(buffers, info_, completed_request->metadata, filename, app_->CameraModel(), options);
		else
			LOG(1, "No still options - unable to save JPEG");
	}

	// Now we'll drop this frame unless it's the last one that we need, at which point
	// we do our HDR processing and send that through.
	frame_num_++;
	if (frame_num_ < config_.num_frames)
		return true;

	// Do HDR processing.
	LOG(1, "Doing HDR processing...");
	acc_.Scale(16.0 / config_.num_frames);

	lp_ = acc_.LpFilter(config_.lp_filter);
	acc_.Tonemap(lp_, config_);

	acc_.Extract(image, info_.stride);
	LOG(1, "HDR done!");

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new HdrStage(app);
}

static RegisterStage reg(NAME, &Create);
