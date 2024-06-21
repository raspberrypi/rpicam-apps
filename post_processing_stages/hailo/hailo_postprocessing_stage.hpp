/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_postprocessing.hpp - Hailo inference postprocessing stage base class.
 */

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <hailo/hailort.hpp>
#include "hailo_objects.hpp"

#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

#include "hailo_postproc_lib.h"

class Allocator
{
public:
	Allocator();
	~Allocator();

	void Reset();

	std::shared_ptr<uint8_t> Allocate(unsigned int size);

private:
	void free(uint8_t *ptr);

	struct AllocInfo
	{
		AllocInfo(uint8_t *_ptr, unsigned int _size, bool _free)
			: ptr(_ptr), size(_size), free(_free)
		{
		}

		uint8_t *ptr;
		unsigned int size;
		bool free;
	};

	std::vector<AllocInfo> alloc_info_;
	std::mutex lock_;
};

class OutTensor
{
public:
	std::shared_ptr<uint8_t> data;
	std::string name;
	hailo_quant_info_t quant_info;
	hailo_3d_image_shape_t shape;
	hailo_format_t format;

	OutTensor(std::shared_ptr<uint8_t> data, const std::string &name, const hailo_quant_info_t &quant_info,
			  const hailo_3d_image_shape_t &shape, hailo_format_t format)
		: data(std::move(data)), name(name), quant_info(quant_info), shape(shape), format(format)
	{
	}

	~OutTensor()
	{
	}

	friend std::ostream &operator<<(std::ostream &os, const OutTensor &t)
	{
		os << "OutTensor: h " << t.shape.height << ", w " << t.shape.width << ", c " << t.shape.features;
		return os;
	}

	static bool SortFunction(const OutTensor &l, const OutTensor &r)
	{
		return l.shape.width < r.shape.width;
	}
};

class HailoPostProcessingStage : public PostProcessingStage
{
public:
	HailoPostProcessingStage(RPiCamApp *app);
	~HailoPostProcessingStage();

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

protected:
	bool Ready() const
	{
		return init_ && low_res_stream_ && output_stream_;
	}

	static std::string PostProcLibDir(const std::string &lib)
	{
		return std::string(HAILO_POSTPROC_LIB_DIR) + "/" + lib;
	}

	const libcamera::Size &InputTensorSize() const
	{
		return input_tensor_size_;
	}

	hailo_status DispatchJob(const uint8_t *input, hailort::AsyncInferJob &job, std::vector<OutTensor> &output_tensors);
	HailoROIPtr MakeROI(const std::vector<OutTensor> &output_tensors) const;

	libcamera::Stream *low_res_stream_;
	libcamera::Stream *output_stream_;
	libcamera::Stream *raw_stream_;
	StreamInfo low_res_info_;
	StreamInfo output_stream_info_;

	Allocator allocator_;

	std::unique_ptr<hailort::VDevice> vdevice_;
	std::shared_ptr<hailort::InferModel> infer_model_;
	std::shared_ptr<hailort::ConfiguredInferModel> configured_infer_model_;

private:
	int configureHailoRT();

	std::mutex lock_;
	bool init_ = false;
	std::string hef_file_;
	hailort::ConfiguredInferModel::Bindings bindings_;
	std::chrono::time_point<std::chrono::steady_clock> last_frame_;
	libcamera::Size input_tensor_size_;
};
