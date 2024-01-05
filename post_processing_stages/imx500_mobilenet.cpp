/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * imx500_mobilenet.cpp - IMX500 inference for MobileNet SSD
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <future>
#include <string>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"
#include "post_processing_stages/object_detect.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

#include "apParams.flatbuffers_generated.h"

using Stream = libcamera::Stream;
namespace controls = libcamera::controls;

#define NAME "imx500_mobilenet"

// Derived from SSDMobilnetV1 DNN Model
static constexpr unsigned int TotalDetections = 10;
// bbox(10*4)+class(10)+scores(10)+numDetections(1) = 61
static constexpr unsigned int DnnOutputTensorSize = 61;
// setup in the IMX500 driver
static constexpr unsigned int TensorStride = 4064;

enum TensorDataType
{
	TYPE_SIGNED = 0,
	TYPE_UNSIGNED
};

struct DnnHeader
{
	uint8_t frameValid;
	uint8_t frameCount;
	uint16_t maxLineLen;
	uint16_t apParamSize;
	uint16_t networkId;
	uint8_t tensorType;
};

struct Dimensions
{
	uint8_t ordinal;
	uint16_t size;
	uint8_t serializationIndex;
	uint8_t padding;
};

struct OutputTensorApParams
{
	uint8_t id;
	char *name;
	uint16_t numDimensions;
	uint8_t bitsPerElement;
	std::vector<Dimensions> vecDim;
	uint16_t shift;
	float scale;
	uint8_t format;
};

struct OutputTensorInfo
{
	std::vector<float> address;
	size_t totalSize;
	uint32_t tensorNum;
	std::vector<uint32_t> tensorDataNum;
};

// MobileNet SSD specific structures
struct Bbox
{
	float x0;
	float y0;
	float x1;
	float y1;
};

struct ObjectDetectionSsdOutputTensor
{
	unsigned int numDetections = 0;
	std::vector<Bbox> bboxes;
	std::vector<float> scores;
	std::vector<float> classes;
};

class MobileNet : public PostProcessingStage
{
public:
	MobileNet(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	int processOutputTensor(std::vector<Detection> &objects, const OutputTensorInfo &outputBodyInfo) const;

	Stream *stream_;

	// Config params
	unsigned int maxDetections_;
	float threshold_;
	std::vector<std::string> classes_;
};

char const *MobileNet::Name() const
{
	return NAME;
}

void MobileNet::Read(boost::property_tree::ptree const &params)
{
	maxDetections_ = params.get<unsigned int>("max_detections");
	threshold_ = params.get<float>("threshold", 0.5f);

	std::string classFile = params.get<std::string>("class_file");
	std::ifstream f(classFile);
	if (f.is_open())
	{
		std::string c;
		while (std::getline(f, c))
			classes_.push_back(c);
	}
	else
		LOG_ERROR("Failed to open class file!");
}

void MobileNet::Configure()
{
	stream_ = app_->GetMainStream();
}

static int parseHeader(DnnHeader &dnnHeader, std::vector<uint8_t> &apParams, const uint8_t *src)
{
	constexpr unsigned int DnnHeaderSize = 12;
	constexpr unsigned int MipiPhSize = 0;

	dnnHeader = *(DnnHeader *)src;

	LOG(2, "Header: valid " << (bool)dnnHeader.frameValid << " count " << (int)dnnHeader.frameCount << " max len "
							<< dnnHeader.maxLineLen << " ap param size " << dnnHeader.apParamSize << " network id "
							<< dnnHeader.networkId << " tensor type " << (int)dnnHeader.tensorType);

	if (!dnnHeader.frameValid)
		return -1;

	apParams.resize(dnnHeader.apParamSize, 0);

	uint32_t i = DnnHeaderSize;
	for (unsigned int j = 0; j < dnnHeader.apParamSize; j++)
	{
		if (i >= TensorStride)
		{
			i = 0;
			src += TensorStride + MipiPhSize;
		}
		apParams[j] = src[i++];
	}

	return 0;
}

int parseApParams(std::vector<OutputTensorApParams> &outputApParams, const std::vector<uint8_t> &apParams,
				  const DnnHeader &dnnHeader)
{
	const apParams::fb::FBApParams *fbApParams;
	const apParams::fb::FBNetwork *fbNetwork;
	const apParams::fb::FBOutputTensor *fbOutputTensor;

	fbApParams = apParams::fb::GetFBApParams(apParams.data());
	LOG(2, "Networks size: " << fbApParams->networks()->size());

	outputApParams.clear();

	for (unsigned int i = 0; i < fbApParams->networks()->size(); i++)
	{
		fbNetwork = (apParams::fb::FBNetwork *)(fbApParams->networks()->Get(i));
		if (fbNetwork->id() != dnnHeader.networkId)
			continue;

		LOG(2, "Network: " << fbNetwork->type()->c_str() << ", i/p size: " << fbNetwork->inputTensors()->size()
						   << ", o/p size: " << fbNetwork->outputTensors()->size());

		for (unsigned int j = 0; j < fbNetwork->outputTensors()->size(); j++)
		{
			OutputTensorApParams outApParam;

			fbOutputTensor = (apParams::fb::FBOutputTensor *)fbNetwork->outputTensors()->Get(j);

			outApParam.id = fbOutputTensor->id();
			outApParam.name = (char *)fbOutputTensor->name()->c_str();
			outApParam.numDimensions = fbOutputTensor->numOfDimensions();

			for (unsigned int k = 0; k < fbOutputTensor->numOfDimensions(); k++)
			{
				Dimensions dim;
				dim.ordinal = fbOutputTensor->dimensions()->Get(k)->id();
				dim.size = fbOutputTensor->dimensions()->Get(k)->size();
				dim.serializationIndex = fbOutputTensor->dimensions()->Get(k)->serializationIndex();
				dim.padding = fbOutputTensor->dimensions()->Get(k)->padding();
				if (dim.padding != 0)
				{
					LOG_ERROR("Error in AP Params, Non-Zero padding for Dimension " << k);
					return -1;
				}

				outApParam.vecDim.push_back(dim);
			}

			outApParam.bitsPerElement = fbOutputTensor->bitsPerElement();
			outApParam.shift = fbOutputTensor->shift();
			outApParam.scale = fbOutputTensor->scale();
			outApParam.format = fbOutputTensor->format();

			/* Add the element to vector */
			outputApParams.push_back(outApParam);
		}

		break;
	}

	return 0;
}

int populateOutputBodyInfo(OutputTensorInfo &outputBodyInfo, const std::vector<OutputTensorApParams> &outputApParams)
{
	// Calculate total output size
	unsigned int totalOutSize = 0;
	for (auto const &ap : outputApParams)
	{
		unsigned int totalDimensionSize = 1;
		for (auto &dim : ap.vecDim)
		{
			if (totalDimensionSize >= UINT32_MAX / dim.size)
			{
				LOG_ERROR("Invalid totalDimensionSize");
				return -1;
			}

			totalDimensionSize *= dim.size;
		}

		if (totalOutSize >= UINT32_MAX - totalDimensionSize)
		{
			LOG_ERROR("Invalid totalOutSize");
			return -1;
		}

		totalOutSize += totalDimensionSize;
	}

	// CodeSonar check
	if (totalOutSize == 0)
	{
		LOG_ERROR("Invalid output tensor info (totalOutSize is 0)");
		return -1;
	}

	LOG(2, "Final output size: " << totalOutSize);

	if (totalOutSize >= UINT32_MAX / sizeof(float))
	{
		LOG_ERROR("Invalid output tensor info");
		return -1;
	}

	// Set FrameOutputTensorInfo
	outputBodyInfo.address.resize(totalOutSize, 0.0f);
	unsigned int numOutputTensors = outputApParams.size();

	/* CodeSonar Check */
	if (!numOutputTensors)
	{
		LOG_ERROR("Invalid numOutputTensors (0)");
		return -1;
	}

	if (numOutputTensors >= UINT32_MAX / sizeof(uint32_t))
	{
		LOG_ERROR("Invalid numOutputTensors");
		return -1;
	}

	outputBodyInfo.totalSize = totalOutSize;
	outputBodyInfo.tensorNum = numOutputTensors;
	outputBodyInfo.tensorDataNum.resize(numOutputTensors, 0);

	return 0;
}

template <typename T>
float getVal8(const uint8_t *src, const OutputTensorApParams &param)
{
	T temp = (T) *src;
	float value = (temp - param.shift) * param.scale;
	return value;
}

template <typename T>
float getVal16(const uint8_t *src, const OutputTensorApParams &param)
{
	T temp = (((T) *(src + 1)) & 0xff) << 8 | (*src & 0xff);
	float value = (temp - param.shift) * param.scale;
	return value;
}

int parseOutputTensorBody(OutputTensorInfo &outputBodyInfo, const uint8_t *src,
						  const std::vector<OutputTensorApParams> &outputApParams, const DnnHeader &dnnHeader)
{
	float *dst = outputBodyInfo.address.data();
	int ret = 0;

	if (outputBodyInfo.totalSize > (UINT32_MAX / sizeof(float)))
	{
		LOG_ERROR("totalSize is greater than maximum size");
		return -1;
	}

	std::vector<float> tmpDst(outputBodyInfo.totalSize, 0.0f);
	std::vector<uint16_t> numLinesVec(outputApParams.size());
	std::vector<uint32_t> outSizes(outputApParams.size());
	std::vector<uint32_t> offsets(outputApParams.size());
	std::vector<const uint8_t *> srcArr(outputApParams.size());
	std::vector<std::vector<Dimensions>> serializedDims;
	std::vector<std::vector<Dimensions>> actualDims;

	const uint8_t *src1 = src;
	uint32_t offset = 0;
	for (unsigned int tensorIdx = 0; tensorIdx < outputApParams.size(); tensorIdx++)
	{
		offsets[tensorIdx] = offset;
		srcArr[tensorIdx] = src1;
		uint32_t tensorDataNum = 0;

		const OutputTensorApParams &param = outputApParams.at(tensorIdx);
		uint32_t outputTensorSize = 0;
		uint32_t tensorOutSize = (param.bitsPerElement / 8);
		std::vector<Dimensions> serializedDim(param.numDimensions);
		std::vector<Dimensions> actualDim(param.numDimensions);

		for (int idx = 0; idx < param.numDimensions; idx++)
		{
			actualDim[idx].size = param.vecDim.at(idx).size;
			serializedDim[param.vecDim.at(idx).serializationIndex].size = param.vecDim.at(idx).size;

			tensorOutSize *= param.vecDim.at(idx).size;
			if (tensorOutSize >= UINT32_MAX / param.bitsPerElement / 8)
			{
				LOG_ERROR("Invalid output tensor info");
				return -1;
			}

			actualDim[idx].serializationIndex = param.vecDim.at(idx).serializationIndex;
			serializedDim[param.vecDim.at(idx).serializationIndex].serializationIndex = (uint8_t)idx;
		}

		uint16_t numLines = (uint16_t)std::ceil(tensorOutSize / (float)dnnHeader.maxLineLen);
		outputTensorSize = tensorOutSize;
		numLinesVec[tensorIdx] = numLines;
		outSizes[tensorIdx] = tensorOutSize;

		serializedDims.push_back(serializedDim);
		actualDims.push_back(actualDim);

		src1 += numLines * TensorStride;
		tensorDataNum = (outputTensorSize / (param.bitsPerElement / 8));
		offset += tensorDataNum;
		outputBodyInfo.tensorDataNum[tensorIdx] = tensorDataNum;
		if (offset > outputBodyInfo.totalSize)
		{
			LOG_ERROR("Error in parsing output tensor offset " << offset << " > output_size");
			return -1;
		}
	}

	std::vector<uint32_t> idxs(outputApParams.size());
	for (unsigned int i = 0; i < idxs.size(); i++)
		idxs[i] = i;

	for (unsigned int i = 0; i < idxs.size(); i++)
	{
		for (unsigned int j = 0; j < idxs.size(); j++)
		{
			if (numLinesVec[idxs[i]] > numLinesVec[idxs[j]])
				std::swap(idxs[i], idxs[j]);
		}
	}

	std::vector<std::future<int>> futures;
	for (unsigned int i = 0; i < idxs.size(); i++)
	{
		uint32_t tensorIdx = idxs[i];
		futures.emplace_back(std::async(std::launch::async,
			[&tmpDst, &outSizes, &numLinesVec, &actualDims, &serializedDims, &outputApParams, &dnnHeader, dst]
				(int tensorIdx, const uint8_t *src, int offset) -> int
			{
				uint32_t outputTensorSize = outSizes[tensorIdx];
				uint16_t numLines = numLinesVec[tensorIdx];
				bool sortingRequired = false;

				const OutputTensorApParams &param = outputApParams[tensorIdx];
				const std::vector<Dimensions> &serializedDim = serializedDims[tensorIdx];
				const std::vector<Dimensions> &actualDim = actualDims[tensorIdx];

				for (int idx = 0; idx < param.numDimensions; idx++)
				{
					if (param.vecDim.at(idx).serializationIndex != param.vecDim.at(idx).ordinal)
						sortingRequired = true;
				}

				if (!outputTensorSize)
				{
					LOG_ERROR("Invalid output tensorsize (0)");
					return -1;
				}

				// Extract output tensor data
				uint32_t elementIndex = 0;
				if (param.bitsPerElement == 8)
				{
					for (unsigned int i = 0; i < numLines; i++)
					{
						int lineIndex = 0;
						while (lineIndex < dnnHeader.maxLineLen)
						{
							if (param.format == TYPE_SIGNED)
								tmpDst[offset + elementIndex] = getVal8<int8_t>(src + lineIndex, param);
							else
								tmpDst[offset + elementIndex] = getVal8<uint8_t>(src + lineIndex, param);
							elementIndex++;
							lineIndex++;
							if (elementIndex == outputTensorSize)
								break;
						}
						src += TensorStride;
						if (elementIndex == outputTensorSize)
							break;
					}
				}
				else if (param.bitsPerElement == 16)
				{
					for (unsigned int i = 0; i < numLines; i++)
					{
						int lineIndex = 0;
						while (lineIndex < dnnHeader.maxLineLen)
						{
							if (param.format == TYPE_SIGNED)
								tmpDst[offset + elementIndex] = getVal16<int16_t>(src + lineIndex, param);
							else
								tmpDst[offset + elementIndex] = getVal16<uint16_t>(src + lineIndex, param);
							elementIndex++;
							lineIndex += 2;
							if (elementIndex >= (outputTensorSize >> 1))
								break;
						}
						src += TensorStride;
						if (elementIndex >= (outputTensorSize >> 1))
							break;
					}
				}

				// Sorting in order according to AP Params. Not supported if larger than 3D
				// Preparation
				if (sortingRequired)
				{
					constexpr unsigned int DimensionMax = 3;

					std::array<uint32_t, DimensionMax> loopCnt { 1, 1, 1 };
					std::array<uint32_t, DimensionMax> coef { 1, 1, 1 };
					for (unsigned int i = 0; i < param.numDimensions; i++)
					{
						if (i >= DimensionMax)
						{
							LOG_ERROR("numDimensions value is 3 or higher");
							break;
						}

						loopCnt[i] = serializedDim.at(i).size;

						for (unsigned int j = serializedDim.at(i).serializationIndex; j > 0; j--)
							coef[i] *= actualDim.at(j - 1).size;
					}
					// Sort execution
					unsigned int src_index = 0;
					unsigned int dst_index;
					for (unsigned int i = 0; i < loopCnt[DimensionMax - 1]; i++)
					{
						for (unsigned int j = 0; j < loopCnt[DimensionMax - 2]; j++)
						{
							for (unsigned int k = 0; k < loopCnt[DimensionMax - 3]; k++)
							{
								dst_index = (coef[DimensionMax - 1] * i) + (coef[DimensionMax - 2] * j) +
											(coef[DimensionMax - 3] * k);
								dst[offset + dst_index] = tmpDst[offset + src_index++];
							}
						}
					}
				}
				else
				{
					if (param.bitsPerElement == 8)
						memcpy(dst + offset, tmpDst.data() + offset, outputTensorSize * sizeof(float));
					else if (param.bitsPerElement == 16)
						memcpy(dst + offset, tmpDst.data() + offset, (outputTensorSize >> 1) * sizeof(float));
					else
					{
						LOG_ERROR("Invalid bitsPerElement value =" << param.bitsPerElement);
						return -1;
					}
				}

				return 0;
			},
			tensorIdx, srcArr[tensorIdx], offsets[tensorIdx]));
	}

	for (auto &f : futures)
		ret += f.get();

	return ret;
}

static int createObjectDetectionSsdData(ObjectDetectionSsdOutputTensor &output, const std::vector<float> &data)
{
	unsigned int count = 0;

	if ((count + (TotalDetections * 4)) > DnnOutputTensorSize)
	{
		LOG_ERROR("Invalid count index " << count);
		return -1;
	}

	// Extract bounding box co-ordinates
	for (unsigned int i = 0; i < TotalDetections; i++)
	{
		Bbox bbox;
		bbox.y0 = data.at(count + i);
		bbox.x0 = data.at(count + i + (1 * TotalDetections));
		bbox.y1 = data.at(count + i + (2 * TotalDetections));
		bbox.x1 = data.at(count + i + (3 * TotalDetections));
		output.bboxes.push_back(bbox);
	}
	count += (TotalDetections * 4);

	// Extract class indices
	for (unsigned int i = 0; i < TotalDetections; i++)
	{
		if (count > DnnOutputTensorSize)
		{
			LOG_ERROR("Invalid count index " << count);
			return -1;
		}

		output.classes.push_back(data.at(count));
		count++;
	}

	// Extract scores
	for (unsigned int i = 0; i < TotalDetections; i++)
	{
		if (count > DnnOutputTensorSize)
		{
			LOG_ERROR("Invalid count index " << count);
			return -1;
		}

		output.scores.push_back(data.at(count));
		count++;
	}

	if (count > DnnOutputTensorSize)
	{
		LOG_ERROR("Invalid count index " << count);
		return -1;
	}

	// Extract number of detections
	unsigned int numDetections = data.at(count);
	if (numDetections > TotalDetections)
	{
		LOG(1, "Unexpected value for numDetections: " << numDetections << ", setting it to " << TotalDetections);
		numDetections = TotalDetections;
	}

	output.numDetections = numDetections;
	return 0;
}

bool MobileNet::Process(CompletedRequestPtr &completed_request)
{
	DnnHeader dnnHeader;
	std::vector<uint8_t> apParams;
	std::vector<OutputTensorApParams> outputApParams;
	OutputTensorInfo outputBodyInfo;

	auto output = completed_request->metadata.get(controls::rpi::Imx500OutputTensor);
	if (!output)
	{
		LOG_ERROR("No output tensor found in metadata!");
		return false;
	}

	const uint8_t *src = output->data();
	int ret = parseHeader(dnnHeader, apParams, src);
	if (ret)
	{
		LOG_ERROR("Header param parsing failed!");
		return false;
	}

	ret = parseApParams(outputApParams, apParams, dnnHeader);
	if (ret)
	{
		LOG_ERROR("AP param parsing failed!");
		return false;
	}

	ret = populateOutputBodyInfo(outputBodyInfo, outputApParams);
	if (ret)
	{
		LOG_ERROR("Failed to populate OutputBodyInfo!");
		return false;
	}

	ret = parseOutputTensorBody(outputBodyInfo, src + TensorStride, outputApParams, dnnHeader);
	if (ret)
	{
		LOG_ERROR("Output tensor body parsing failed!");
		return false;
	}

	std::vector<Detection> objects;
	ret = processOutputTensor(objects, outputBodyInfo);
	if (!ret && objects.size())
		completed_request->post_process_metadata.Set("object_detect.results", objects);

	return false;
}

int MobileNet::processOutputTensor(std::vector<Detection> &objects, const OutputTensorInfo &outputBodyInfo) const
{
	ObjectDetectionSsdOutputTensor tensor;

	if (outputBodyInfo.totalSize != DnnOutputTensorSize)
	{
		LOG_ERROR("Invalid totalSize " << outputBodyInfo.totalSize);
		return -1;
	}

	int ret = createObjectDetectionSsdData(tensor, outputBodyInfo.address);
	if (ret)
	{
		LOG_ERROR("Failed to create SSD data");
		return -1;
	}

	const libcamera::Size dim = stream_->configuration().size;
	for (unsigned int i = 0; i < std::min(tensor.numDetections, maxDetections_); i++)
	{
		uint8_t classIndex = (uint8_t)tensor.classes[i];

		// Filter detections
		if (tensor.scores[i] < threshold_ || classIndex >= classes_.size())
			continue;

		// Extract bounding box co-ordinates
		unsigned int x0 = std::round((tensor.bboxes[i].x0) * (dim.width - 1));
		unsigned int x1 = std::round((tensor.bboxes[i].x1) * (dim.width - 1));
		unsigned int y0 = std::round((tensor.bboxes[i].y0) * (dim.height - 1));
		unsigned int y1 = std::round((tensor.bboxes[i].y1) * (dim.height - 1));

		objects.emplace_back(classIndex, classes_[classIndex], tensor.scores[i], x0, y0, x1 - x0, y1 - y0);
	}

	LOG(1, "Number of objects detected: " << objects.size());
	for (unsigned i = 0; i < objects.size(); i++)
		LOG(2, "[" << i << "] : " << objects[i].toString());

	return 0;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new MobileNet(app);
}

static RegisterStage reg(NAME, &Create);
