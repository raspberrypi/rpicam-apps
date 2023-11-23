/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * segmentation_tf_stage - image segmentation
 */

#include "segmentation.hpp"
#include "tf_stage.hpp"

// The neural network expects a 257x257 input.

constexpr int WIDTH = 257;
constexpr int HEIGHT = 257;

struct SegmentationTfConfig : public TfConfig
{
	bool draw;
	uint32_t threshold; // number of pixels in a category before we print its name
};

#define NAME "segmentation_tf"

class SegmentationTfStage : public TfStage
{
public:
	SegmentationTfStage(RPiCamApp *app) : TfStage(app, WIDTH, HEIGHT), segmentation_(WIDTH * HEIGHT)
	{
		config_ = std::make_unique<SegmentationTfConfig>();
	}
	char const *Name() const override { return NAME; }

protected:
	SegmentationTfConfig *config() const { return static_cast<SegmentationTfConfig *>(config_.get()); }

	void readExtras(boost::property_tree::ptree const &params) override;

	void checkConfiguration() override;

	// Read out the segmentation map.
	void interpretOutputs() override;

	// Add metadata and optionally draw the segmentation in the corner of the image.
	void applyResults(CompletedRequestPtr &completed_request) override;

	void readLabelsFile(const std::string &filename);

private:
	std::vector<std::string> labels_;
	std::vector<uint8_t> segmentation_;
};

void SegmentationTfStage::readLabelsFile(const std::string &file_name)
{
	std::ifstream file(file_name);
	if (!file)
		throw std::runtime_error("SegmentationTfStage: Failed to load labels file");

	for (std::string line; std::getline(file, line); labels_.push_back(line))
		;
}

void SegmentationTfStage::readExtras([[maybe_unused]] boost::property_tree::ptree const &params)
{
	config()->draw = params.get<int>("draw", 1);
	config()->threshold = params.get<uint32_t>("threshold", 5000);
	std::string labels_file = params.get<std::string>("labels_file", "");
	readLabelsFile(labels_file);

	// Check the output dimensions.
	int output = interpreter_->outputs()[0];
	TfLiteIntArray *dims = interpreter_->tensor(output)->dims;
	if (dims->size != 4 || dims->data[1] != HEIGHT || dims->data[2] != WIDTH ||
		dims->data[3] != static_cast<int>(labels_.size()))
		throw std::runtime_error("SegmentationTfStage: Unexpected output tensor size");
}

void SegmentationTfStage::checkConfiguration()
{
	if (!main_stream_ && config()->draw)
		throw std::runtime_error("SegmentationTfStage: Main stream is required for drawing");
}

void SegmentationTfStage::applyResults(CompletedRequestPtr &completed_request)
{
	// Store the segmentation in image metadata.
	completed_request->post_process_metadata.Set("segmentation.result",
												Segmentation(WIDTH, HEIGHT, labels_, segmentation_));

	// Optionally, draw the segmentation in the bottom right corner of the main image.
	if (!config()->draw)
		return;

	BufferWriteSync w(app_, completed_request->buffers[main_stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	int y_offset = main_stream_info_.height - HEIGHT;
	int x_offset = main_stream_info_.width - WIDTH;
	int scale = 255 / labels_.size();

	for (int y = 0; y < HEIGHT; y++)
	{
		uint8_t *src = &segmentation_[y * WIDTH];
		uint8_t *dst = buffer.data() + (y + y_offset) * main_stream_info_.stride + x_offset;
		for (int x = 0; x < WIDTH; x++)
			*(dst++) = scale * *(src++);
	}

	// Also make it greyscale.
	uint8_t *U_start = buffer.data() + main_stream_info_.height * main_stream_info_.stride;
	int UV_size = (main_stream_info_.height / 2) * (main_stream_info_.stride / 2);
	y_offset /= 2;
	x_offset /= 2;

	for (int y = 0; y < HEIGHT / 2; y++)
	{
		uint8_t *dst = U_start + (y + y_offset) * (main_stream_info_.stride / 2) + x_offset;
		memset(dst, 128, WIDTH / 2);
		memset(dst + UV_size, 128, WIDTH / 2);
	}
}

void SegmentationTfStage::interpretOutputs()
{
	float *output = interpreter_->tensor(interpreter_->outputs()[0])->data.f;
	uint8_t *seg_ptr = &segmentation_[0];
	int num_categories = labels_.size();
	std::vector<std::pair<size_t, int>> hist(num_categories);
	std::generate(hist.begin(), hist.end(), [i = 0]() mutable { return std::pair<size_t, int>(0, i++); });

	// Extract the segmentation from the output tensor. Also accumulate a histogram.

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++, output += num_categories)
		{
			// For each pixel we get a "confidence" value for every category - pick the largest.
			int index = std::max_element(output, output + num_categories) - output;
			*(seg_ptr++) = index;
			hist[index].first++;
		}
	}

	if (config()->verbose)
	{
		// Output the category names of the largest histogram bins.
		std::sort(hist.begin(), hist.end(), [](const auto &lhs, const auto &rhs) { return lhs.first > rhs.first; });
		for (int i = 0; i < num_categories && hist[i].first >= config()->threshold; i++)
			std::cerr << (i ? ", " : "") << labels_[hist[i].second] << " (" << hist[i].first << ")";
		std::cerr << std::endl;
	}
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new SegmentationTfStage(app);
}

static RegisterStage reg(NAME, &Create);

