/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * tf_stage.hpp - base class for TensorFlowLite stages
 */

#include "tf_stage.hpp"

TfStage::TfStage(LibcameraApp *app, int tf_w, int tf_h) : PostProcessingStage(app), tf_w_(tf_w), tf_h_(tf_h)
{
	if (tf_w_ <= 0 || tf_h_ <= 0)
		throw std::runtime_error("TfStage: Bad TFLite input dimensions");
}

void TfStage::Read(boost::property_tree::ptree const &params)
{
	config_->number_of_threads = params.get<int>("number_of_threads", 2);
	config_->refresh_rate = params.get<int>("refresh_rate", 5);
	config_->model_file = params.get<std::string>("model_file", "");
	config_->verbose = params.get<int>("verbose", 0);
	config_->normalisation_offset = params.get<float>("normalisation_offset", 127.5);
	config_->normalisation_scale = params.get<float>("normalisation_scale", 127.5);

	initialise();

	readExtras(params);
}

void TfStage::initialise()
{
	model_ = tflite::FlatBufferModel::BuildFromFile(config_->model_file.c_str());
	if (!model_)
		throw std::runtime_error("TfStage: Failed to load model");
	std::cout << "TfStage: Loaded model " << config_->model_file << std::endl;

	tflite::ops::builtin::BuiltinOpResolver resolver;
	tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
	if (!interpreter_)
		throw std::runtime_error("TfStage: Failed to construct interpreter");

	if (config_->number_of_threads != -1)
		interpreter_->SetNumThreads(config_->number_of_threads);

	if (interpreter_->AllocateTensors() != kTfLiteOk)
		throw std::runtime_error("TfStage: Failed to allocate tensors");

	// Make an attempt to verify that the model expects this size of input.
	int input = interpreter_->inputs()[0];
	size_t size = interpreter_->tensor(input)->bytes;
	size_t check = tf_w_ * tf_h_ * 3; // assume RGB
	if (interpreter_->tensor(input)->type == kTfLiteUInt8)
		check *= sizeof(uint8_t);
	else if (interpreter_->tensor(input)->type == kTfLiteFloat32)
		check *= sizeof(float);
	else
		throw std::runtime_error("TfStage: Input tensor data type not supported");

	// Causes might include loading the wrong model.
	if (check != size)
		throw std::runtime_error("TfStage: Input tensor size mismatch");
}

void TfStage::Configure()
{
	lores_w_ = lores_h_ = lores_stride_ = 0;
	lores_stream_ = app_->LoresStream();
	if (lores_stream_)
	{
		app_->StreamDimensions(lores_stream_, &lores_w_, &lores_h_, &lores_stride_);
		if (config_->verbose)
			std::cout << "TfStage: Low resolution stream is " << lores_w_ << "x" << lores_h_ << std::endl;
		if (tf_w_ > lores_w_ || tf_h_ > lores_h_)
		{
			std::cout << "TfStage: WARNING: Low resolution image too small" << std::endl;
			lores_stream_ = nullptr;
		}
	}
	else if (config_->verbose)
		std::cout << "TfStage: no low resolution stream" << std::endl;

	main_w_ = main_h_ = main_stride_ = 0;
	main_stream_ = app_->GetMainStream();
	if (main_stream_)
	{
		app_->StreamDimensions(main_stream_, &main_w_, &main_h_, &main_stride_);
		if (config_->verbose)
			std::cout << "TfStage: Main stream is " << main_w_ << "x" << main_h_ << std::endl;
	}
	else if (config_->verbose)
		std::cout << "TfStage: No main stream" << std::endl;

	checkConfiguration();
}

bool TfStage::Process(CompletedRequest &completed_request)
{
	if (!lores_stream_)
		return false;

	{
		std::unique_lock<std::mutex> lck(future_mutex_);
		if (config_->refresh_rate && completed_request.sequence % config_->refresh_rate == 0 &&
			(!future_ || future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready))
		{
			libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request.buffers[lores_stream_])[0];

			// Copy the lores image here and let the asynchronous thread convert it to RGB.
			// Doing the "extra" copy is in fact hugely beneficial because it turns uncacned
			// memory into cached memory, which is then *much* quicker.
			lores_copy_.assign(buffer.data(), buffer.data() + buffer.size());

			future_ = std::make_unique<std::future<void>>();
			*future_ = std::move(std::async(std::launch::async, [this] {
				auto time_taken = ExecutionTime<std::micro>(&TfStage::runInference, this).count();

				if (config_->verbose)
					std::cout << "TfStage: Inference time: " << time_taken << " ms" << std::endl;
			}));
		}
	}

	std::unique_lock<std::mutex> lock(output_mutex_);
	applyResults(completed_request);

	return false;
}

std::vector<uint8_t> TfStage::yuvToRgb(const uint8_t *src, int src_w, int src_h, int src_stride, int dst_w, int dst_h,
									   int dst_stride)
{
	std::vector<uint8_t> output(dst_h * dst_stride);

	assert(src_w >= dst_w && src_h >= dst_h);
	int off_x = ((src_w - dst_w) / 2) & ~1, off_y = ((src_h - dst_h) / 2) & ~1;
	int src_Y_size = src_h * src_stride, src_U_size = (src_h / 2) * (src_stride / 2);

	// We're going to process 4x2 pixel blocks, as far as alignment allows.
	int dst_h_aligned = dst_h & ~1, dst_w_aligned = dst_w & ~3;

	int y = 0;
	for (; y < dst_h_aligned; y += 2)
	{
		const uint8_t *src_Y0 = src + (y + off_y) * src_stride + off_x;
		const uint8_t *src_U = src + src_Y_size + ((y + off_y) / 2) * (src_stride / 2) + off_x / 2;
		const uint8_t *src_V = src_U + src_U_size;
		const uint8_t *src_Y1 = src_Y0 + src_stride;
		uint8_t *dst0 = &output[y * dst_stride];
		uint8_t *dst1 = dst0 + dst_stride;

		int x = 0;
		for (; x < dst_w_aligned; x += 4)
		{
			int Y0 = *(src_Y0++);
			int U0 = *(src_U++);
			int V0 = *(src_V++);
			int Y1 = *(src_Y0++);
			int Y2 = *(src_Y0++);
			int U2 = *(src_U++);
			int V2 = *(src_V++);
			int Y3 = *(src_Y0++);
			int Y4 = *(src_Y1++);
			int Y5 = *(src_Y1++);
			int Y6 = *(src_Y1++);
			int Y7 = *(src_Y1++);

			U0 -= 128;
			V0 -= 128;
			U2 -= 128;
			V2 -= 128;
			int U1 = U0;
			int V1 = V0;
			int U4 = U0;
			int V4 = V0;
			int U5 = U0;
			int V5 = V0;
			int U3 = U2;
			int V3 = V2;
			int U6 = U2;
			int V6 = V2;
			int U7 = U2;
			int V7 = V2;

			int R0 = Y0 + 1.402 * V0;
			int G0 = Y0 - 0.345 * U0 - 0.714 * V0;
			int B0 = Y0 + 1.771 * U0;
			int R1 = Y1 + 1.402 * V1;
			int G1 = Y1 - 0.345 * U1 - 0.714 * V1;
			int B1 = Y1 + 1.771 * U1;
			int R2 = Y2 + 1.402 * V2;
			int G2 = Y2 - 0.345 * U2 - 0.714 * V2;
			int B2 = Y2 + 1.771 * U2;
			int R3 = Y3 + 1.402 * V3;
			int G3 = Y3 - 0.345 * U3 - 0.714 * V3;
			int B3 = Y3 + 1.771 * U3;
			int R4 = Y4 + 1.402 * V4;
			int G4 = Y4 - 0.345 * U4 - 0.714 * V4;
			int B4 = Y4 + 1.771 * U4;
			int R5 = Y5 + 1.402 * V5;
			int G5 = Y5 - 0.345 * U5 - 0.714 * V5;
			int B5 = Y5 + 1.771 * U5;
			int R6 = Y6 + 1.402 * V6;
			int G6 = Y6 - 0.345 * U6 - 0.714 * V6;
			int B6 = Y6 + 1.771 * U6;
			int R7 = Y7 + 1.402 * V7;
			int G7 = Y7 - 0.345 * U7 - 0.714 * V7;
			int B7 = Y7 + 1.771 * U7;

			R0 = std::clamp(R0, 0, 255);
			G0 = std::clamp(G0, 0, 255);
			B0 = std::clamp(B0, 0, 255);
			R1 = std::clamp(R1, 0, 255);
			G1 = std::clamp(G1, 0, 255);
			B1 = std::clamp(B1, 0, 255);
			R2 = std::clamp(R2, 0, 255);
			G2 = std::clamp(G2, 0, 255);
			B2 = std::clamp(B2, 0, 255);
			R3 = std::clamp(R3, 0, 255);
			G3 = std::clamp(G3, 0, 255);
			B3 = std::clamp(B3, 0, 255);
			R4 = std::clamp(R4, 0, 255);
			G4 = std::clamp(G4, 0, 255);
			B4 = std::clamp(B4, 0, 255);
			R5 = std::clamp(R5, 0, 255);
			G5 = std::clamp(G5, 0, 255);
			B5 = std::clamp(B5, 0, 255);
			R6 = std::clamp(R6, 0, 255);
			G6 = std::clamp(G6, 0, 255);
			B6 = std::clamp(B6, 0, 255);
			R7 = std::clamp(R7, 0, 255);
			G7 = std::clamp(G7, 0, 255);
			B7 = std::clamp(B7, 0, 255);

			*(dst0++) = R0;
			*(dst0++) = G0;
			*(dst0++) = B0;
			*(dst0++) = R1;
			*(dst0++) = G1;
			*(dst0++) = B1;
			*(dst0++) = R2;
			*(dst0++) = G2;
			*(dst0++) = B2;
			*(dst0++) = R3;
			*(dst0++) = G3;
			*(dst0++) = B3;
			*(dst1++) = R4;
			*(dst1++) = G4;
			*(dst1++) = B4;
			*(dst1++) = R5;
			*(dst1++) = G5;
			*(dst1++) = B5;
			*(dst1++) = R6;
			*(dst1++) = G6;
			*(dst1++) = B6;
			*(dst1++) = R7;
			*(dst1++) = G7;
			*(dst1++) = B7;
		}
		// Straggling pixel columns - we must still do both rows.
		for (; x < dst_w; x++)
		{
			int Y0 = *(src_Y0++);
			int U0 = *(src_U);
			int V0 = *(src_V);
			int Y4 = *(src_Y1++);
			src_U += (x & 1);
			src_V += (x & 1);

			U0 -= 128;
			V0 -= 128;
			int U4 = U0;
			int V4 = V0;

			int R0 = Y0 + 1.402 * V0;
			int G0 = Y0 - 0.345 * U0 - 0.714 * V0;
			int B0 = Y0 + 1.771 * U0;
			int R4 = Y4 + 1.402 * V4;
			int G4 = Y4 - 0.345 * U4 - 0.714 * V4;
			int B4 = Y4 + 1.771 * U4;

			R0 = std::clamp(R0, 0, 255);
			G0 = std::clamp(G0, 0, 255);
			B0 = std::clamp(B0, 0, 255);
			R4 = std::clamp(R4, 0, 255);
			G4 = std::clamp(G4, 0, 255);
			B4 = std::clamp(B4, 0, 255);

			*(dst0++) = R0;
			*(dst0++) = G0;
			*(dst0++) = B0;
			*(dst1++) = R4;
			*(dst1++) = G4;
			*(dst1++) = B4;
		}
	}
	// Any straggling final row is done with extreme steam power.
	for (; y < dst_h; y++)
	{
		const uint8_t *src_Y0 = src + (y + off_y) * src_stride + off_x;
		const uint8_t *src_U = src + src_Y_size + ((y + off_y) / 2) * (src_stride / 2) + off_x / 2;
		const uint8_t *src_V = src_U + src_U_size;
		uint8_t *dst0 = &output[y * dst_stride];

		int x = 0;
		for (; x < dst_w; x++)
		{
			int Y0 = *(src_Y0++);
			int U0 = *(src_U);
			int V0 = *(src_V);
			src_U += (x & 1);
			src_V += (x & 1);

			U0 -= 128;
			V0 -= 128;

			int R0 = Y0 + 1.402 * V0;
			int G0 = Y0 - 0.345 * U0 - 0.714 * V0;
			int B0 = Y0 + 1.771 * U0;

			R0 = std::clamp(R0, 0, 255);
			G0 = std::clamp(G0, 0, 255);
			B0 = std::clamp(B0, 0, 255);

			*(dst0++) = R0;
			*(dst0++) = G0;
			*(dst0++) = B0;
		}
	}

	return output;
}

void TfStage::runInference()
{
	int input = interpreter_->inputs()[0];
	std::vector<uint8_t> rgb_image =
		yuvToRgb(lores_copy_.data(), lores_w_, lores_h_, lores_stride_, tf_w_, tf_h_, tf_w_ * 3);

	if (interpreter_->tensor(input)->type == kTfLiteUInt8)
	{
		uint8_t *tensor = interpreter_->typed_tensor<uint8_t>(input);
		for (int i = 0; i < rgb_image.size(); i++)
			tensor[i] = rgb_image[i];
	}
	else if (interpreter_->tensor(input)->type == kTfLiteFloat32)
	{
		float *tensor = interpreter_->typed_tensor<float>(input);
		for (int i = 0; i < rgb_image.size(); i++)
			tensor[i] = (rgb_image[i] - config_->normalisation_offset) / config_->normalisation_scale;
	}

	if (interpreter_->Invoke() != kTfLiteOk)
		throw std::runtime_error("TfStage: Failed to invoke TFLite");

	std::unique_lock<std::mutex> lock(output_mutex_);
	interpretOutputs();
}

void TfStage::Stop()
{
	if (future_)
		future_->wait();
}
