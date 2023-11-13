/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * tf_stage.hpp - base class for TensorFlowLite stages
 */

#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <libcamera/stream.h>

#include "tensorflow/lite/builtin_op_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

// The TfStage is a convenient base class from which post processing stages using
// TensorFlowLite can be derived. It provides a certain amount of boiler plate code
// and some other useful functions. Please refer to the examples provided that make
// use of it.

struct TfConfig
{
	int number_of_threads = 3;
	int refresh_rate = 5;
	std::string model_file;
	bool verbose = false;
	float normalisation_offset = 127.5;
	float normalisation_scale = 127.5;
};

class TfStage : public PostProcessingStage
{
public:
	// The TfStage provides implementations of the PostProcessingStage functions with the
	// exception of Name(), which derived classes must still provide.

	// The constructor supplies the width and height that TFLite wants.
	TfStage(RPiCamApp *app, int tf_w, int tf_h);

	//char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	void Stop() override;

protected:
	TfConfig *config() const { return config_.get(); }

	// Instead of redefining the above public interface, derived class should implement
	// the following four virtual methods.

	// Read additional parameters required by the stage. Can also do some model checking.
	virtual void readExtras(boost::property_tree::ptree const &params) {}

	// Check the stream and image configuration. Here the stage should report any errors
	// and/or fail.
	virtual void checkConfiguration() {}

	// This runs asynchronously from the main thread right after the model has run. The
	// outputs should be processed into a form where applyResults can make use of them.
	virtual void interpretOutputs() {}

	// Here we run synchronously again and so should not take too long. The results
	// produced by interpretOutputs can be used now, for example as metadata to attach
	// to the image, or even drawn onto the image itself.
	virtual void applyResults(CompletedRequestPtr &completed_request) {}

	std::unique_ptr<TfConfig> config_;

	// The width and height that TFLite wants.
	unsigned int tf_w_, tf_h_;

	// We run TFLite on the low resolution image, details of which are here.
	libcamera::Stream *lores_stream_;
	StreamInfo lores_info_;

	// The stage may or may not make use of the larger or "main" image stream.
	libcamera::Stream *main_stream_;
	StreamInfo main_stream_info_;

	std::unique_ptr<tflite::FlatBufferModel> model_;
	std::unique_ptr<tflite::Interpreter> interpreter_;

private:
	void initialise();
	void runInference();

	std::mutex future_mutex_;
	std::unique_ptr<std::future<void>> future_;
	std::vector<uint8_t> lores_copy_;
	std::mutex output_mutex_;
};
