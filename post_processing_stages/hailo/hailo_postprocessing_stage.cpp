/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * hailo_postprocessing_stage.cpp - Hailo software stage base class and helpers
 */

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <sys/mman.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "core/dl_lib.hpp"

#include "hailo_postprocessing_stage.hpp"

#include "hailo_postproc_lib.h"

using namespace hailort;

using Rectangle = libcamera::Rectangle;
using Size = libcamera::Size;

namespace
{

class Display
{
public:
	static MessageQueue &GetDisplayMsgQueue()
	{
		return GetInstance().msg_queue_;
	}

	static void SetDisplayConfig(int width, int height, bool fullscreen)
	{
		Display &instance = GetInstance();
		instance.window_width_ = width;
		instance.window_height_ = height;
		instance.fullscreen_ = fullscreen;
	}

	static void ToggleFullscreen()
	{
		Display &instance = GetInstance();
		instance.fullscreen_ = !instance.fullscreen_;
	}

private:
	static Display &GetInstance()
	{
		static Display display_instance;
		return display_instance;
	}

	Display()
	{
		display_thread_ = std::thread(&Display::displayThread, this);
		init_ = true;
	}

	MessageQueue msg_queue_;

	~Display()
	{
		if (init_)
		{
			msg_queue_.Post(Msg(MsgType::Quit));
			display_thread_.join();
		}
	}

	void displayThread();

	std::thread display_thread_;
	bool init_ = false;
	int window_width_ = 320;
	int window_height_ = 320;
	bool fullscreen_ = false;
};

// Singleton class for the hardware virtual device.
class vdevice
{
public:
	vdevice(vdevice &other) = delete;
	void operator=(const vdevice &) = delete;

	static VDevice *get_instance()
	{
		static std::unique_ptr<VDevice> _vdevice {};

		if (!_vdevice)
		{
			Expected<std::unique_ptr<VDevice>> vdevice_exp = VDevice::create();
			if (!vdevice_exp)
			{
				LOG_ERROR("Failed create vdevice, status = " << vdevice_exp.status());
				return nullptr;
			}
			_vdevice = vdevice_exp.release();
		}

		return _vdevice.get();
	}

private:
	vdevice() {}
};

// Sigh :(
std::string get_hailo_architecture()
{
	const std::string cmd("hailortcli fw-control identify");
	const std::string target_label("Device Architecture: ");
	std::array<char, 128> buffer;

	auto deleter = [](FILE *f) { pclose(f); };
	std::unique_ptr<FILE, decltype(deleter)> pipe(popen(cmd.c_str(), "r"), deleter);

	if (!pipe)
	{
		LOG_ERROR("Could not open pipe for Hailo identify");
		return {};
	}

	while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
	{
		std::string line(buffer.data());
		size_t pos = line.find(target_label);
		if (pos != std::string::npos)
		{
			std::string arch = line.substr(pos + target_label.length());
			arch.erase(arch.find_last_not_of(" \n\r\t") + 1);
			return arch;
		}
	}

	return {};
}

} // namespace


Allocator::Allocator()
{
}

Allocator::~Allocator()
{
	Reset();
}

void Allocator::Reset()
{
	std::scoped_lock<std::mutex> l(lock_);

	for (auto &info : alloc_info_)
		munmap(info.ptr, info.size);

	alloc_info_.clear();
}

std::shared_ptr<uint8_t> Allocator::Allocate(unsigned int size)
{
	std::scoped_lock<std::mutex> l(lock_);
	uint8_t *ptr = nullptr;

	auto info = std::find_if(alloc_info_.begin(), alloc_info_.end(),
							 [size](const AllocInfo &info) { return info.free && info.size == size; });
	if (info != alloc_info_.end())
	{
		info->free = false;
		ptr = info->ptr;
	}

	if (!ptr)
	{
		void *addr = mmap(NULL, size, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
		if (addr == MAP_FAILED)
			return {};

		ptr = static_cast<uint8_t *>(addr);
		alloc_info_.emplace_back(ptr, size, false);
	}

	return std::shared_ptr<uint8_t>(ptr, [this](uint8_t *ptr) { this->free(ptr); });
}

void Allocator::free(uint8_t *ptr)
{
	std::scoped_lock<std::mutex> l(lock_);

	auto info =	std::find_if(alloc_info_.begin(), alloc_info_.end(),
							 [ptr](const AllocInfo &info) { return info.ptr == ptr; });
	if (info != alloc_info_.end())
		info->free = true;
}

HailoPostProcessingStage::HailoPostProcessingStage(RPiCamApp *app)
	: PostProcessingStage(app), msg_queue_(std::ref(Display::GetDisplayMsgQueue()))
{
}

HailoPostProcessingStage::~HailoPostProcessingStage()
{
	if (init_)
		configured_infer_model_->shutdown();
}

void HailoPostProcessingStage::Read(boost::property_tree::ptree const &params)
{
	hef_file_ = params.get<std::string>("hef_file", "");
	hef_file_8_ = params.get<std::string>("hef_file_8", "");
	hef_file_8L_ = params.get<std::string>("hef_file_8L", "");
	hef_file_10_ = params.get<std::string>("hef_file_10", "");
	display_window_width_ = params.get<int>("display_window_width", 320);
	display_window_height_ = params.get<int>("display_window_height", 320);
	display_fullscreen_ = params.get<bool>("display_fullscreen", false);
}

void HailoPostProcessingStage::Configure()
{
	output_stream_ = app_->GetMainStream();
	raw_stream_ = app_->RawStream();
	low_res_stream_ = app_->LoresStream();
	if (low_res_stream_)
		low_res_info_ = app_->GetStreamInfo(low_res_stream_);
	if (output_stream_)
		output_stream_info_ = app_->GetStreamInfo(output_stream_);

	if (!init_)
	{
		if (!configureHailoRT())
			init_ = true;
	}

	Display::SetDisplayConfig(display_window_width_, display_window_height_, display_fullscreen_);

	allocator_.Reset();
	last_frame_ = {};
}

int HailoPostProcessingStage::configureHailoRT()
{
	vdevice_ = vdevice::get_instance();
	if (!vdevice_)
	{
		LOG_ERROR("Failed to get a vdevice instance.");
		return -1;
	}

	std::string device = get_hailo_architecture();
	if (device.empty())
	{
		LOG_ERROR("Defaulting to HAILO8 architecture");
		device = "HAILO8";
	}
	else
		LOG(1, "Hailo device: " << device);

	std::string hef_file;
	if (device == "HAILO10H")
		hef_file = hef_file_10_;
	else if (device == "HAILO8")
		hef_file = hef_file_8_;
	else if (device == "HAILO8L")
		hef_file = hef_file_8L_;
	else
		LOG_ERROR("Unexpected Hailo architecture detected: " << device);

	if (hef_file.empty())
		hef_file = hef_file_;

	if (hef_file.empty())
	{
		LOG_ERROR("Unable to use a suitable HEF file.");
		return -1;
	}

	// Create infer model from HEF file.
	Expected<std::shared_ptr<InferModel>> infer_model_exp = vdevice_->create_infer_model(hef_file);
	if (!infer_model_exp)
	{
		LOG_ERROR("Failed to create infer model, status = " << infer_model_exp.status());
		return infer_model_exp.status();
	}
	infer_model_ = infer_model_exp.release();
	infer_model_->set_hw_latency_measurement_flags(HAILO_LATENCY_MEASURE);

	// Configure the infer model
	//infer_model_->output()->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);
	Expected<ConfiguredInferModel> configured_infer_model_exp = infer_model_->configure();
	if (!configured_infer_model_exp)
	{
		LOG_ERROR("Failed to create configured infer model, status = " << configured_infer_model_exp.status());
		return configured_infer_model_exp.status();
	}
	configured_infer_model_ = std::make_shared<ConfiguredInferModel>(configured_infer_model_exp.release());

	// Create infer bindings
	Expected<ConfiguredInferModel::Bindings> bindings_exp = configured_infer_model_->create_bindings();
	if (!bindings_exp)
	{
		LOG_ERROR("Failed to create infer bindings, status = " << bindings_exp.status());
		return bindings_exp.status();
	}
	bindings_ = std::move(bindings_exp.release());

	hailo_3d_image_shape_t shape = infer_model_->inputs()[0].shape();
	input_tensor_size_ = libcamera::Size(shape.width, shape.height);

	return 0;
}

hailo_status HailoPostProcessingStage::DispatchJob(const uint8_t *input, AsyncInferJob &job,
												   std::vector<OutTensor> &output_tensors)
{
	hailo_status status;

	std::scoped_lock<std::mutex> l(lock_);

	// Input tensor.
	const std::string &input_name = infer_model_->get_input_names()[0];
	size_t input_frame_size = infer_model_->input(input_name)->get_frame_size();

	status = bindings_.input(input_name)->set_buffer(MemoryView((void *)(input), input_frame_size));
	if (status != HAILO_SUCCESS)
	{
		LOG_ERROR("Could not write to input stream with status " << status);
		return status;
	}

	// Output tensors.
	for (auto const &output_name : infer_model_->get_output_names())
	{
		size_t output_size = infer_model_->output(output_name)->get_frame_size();
		std::shared_ptr<uint8_t> output_buffer = allocator_.Allocate(output_size);
		if (!output_buffer)
		{
			LOG_ERROR("Could not allocate an output buffer!");
			return status;
		}

		status = bindings_.output(output_name)->set_buffer(MemoryView(output_buffer.get(), output_size));
		if (status != HAILO_SUCCESS)
		{
			LOG_ERROR("Failed to set infer output buffer, status = " << status);
			return status;
		}

		const std::vector<hailo_quant_info_t> quant = infer_model_->output(output_name)->get_quant_infos();
		const hailo_3d_image_shape_t shape = infer_model_->output(output_name)->shape();
		const hailo_format_t format = infer_model_->output(output_name)->format();
		output_tensors.emplace_back(std::move(output_buffer), output_name, quant[0], shape, format);
	}

	// Waiting for available requests in the pipeline.
	status = configured_infer_model_->wait_for_async_ready(1s);
	if (status != HAILO_SUCCESS)
	{
		LOG_ERROR("Failed to wait for async ready, status = " << status);
		return status;
	}

	Expected<LatencyMeasurementResult> inf_time_exp = configured_infer_model_->get_hw_latency_measurement();
	std::chrono::time_point<std::chrono::steady_clock> this_frame = std::chrono::steady_clock::now();

	if (inf_time_exp && last_frame_.time_since_epoch() != 0s)
	{
		const auto inf_time =
			std::chrono::duration_cast<std::chrono::milliseconds>(inf_time_exp.release().avg_hw_latency);
		const auto frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(this_frame - last_frame_);

		if (frame_time < inf_time)
			LOG(2, "Warning: model inferencing time of " << inf_time.count() << "ms " <<
				   "> current job interval of " << frame_time.count() << "ms!");
	}

	last_frame_ = this_frame;

	// Dispatch the job.
	Expected<AsyncInferJob> job_exp = configured_infer_model_->run_async(bindings_);
	if (!job_exp)
	{
		LOG_ERROR("Failed to start async infer job, status = " << job_exp.status());
		return status;
	}
	job = job_exp.release();

	// Detach and let the job run.
	job.detach();

	return status;
}

HailoROIPtr HailoPostProcessingStage::MakeROI(const std::vector<OutTensor> &output_tensors) const
{
	HailoROIPtr roi = std::make_shared<HailoROI>(HailoROI(HailoBBox(0.0f, 0.0f, 1.0f, 1.0f)));

	for (auto const &t : output_tensors)
	{
		hailo_tensor_metadata_t info;

		strncpy(info.name, t.name.c_str(), sizeof(info.name));
		// To keep GCC quiet...
		info.name[HAILO_MAX_STREAM_NAME_SIZE - 1] = '\0';
		info.format.type = (HailoTensorFormatType)t.format.type;
		info.format.is_nms = infer_model_->outputs()[0].is_nms();
		info.quant_info.qp_zp = t.quant_info.qp_zp;
		info.quant_info.qp_scale = t.quant_info.qp_scale;
		info.quant_info.limvals_min = t.quant_info.limvals_min;
		info.quant_info.limvals_max = t.quant_info.limvals_max;
		if (info.format.is_nms)
		{
			auto i = infer_model_->outputs()[0].get_nms_shape().release();
			info.nms_shape.number_of_classes = i.number_of_classes;
			info.nms_shape.max_bboxes_per_class = i.max_bboxes_per_class;
			info.nms_shape.max_bboxes_total = i.max_bboxes_total;
			info.nms_shape.max_accumulated_mask_size = i.max_accumulated_mask_size;
		}
		else
		{
			info.shape.height = t.shape.height;
			info.shape.width = t.shape.width;
			info.shape.features = t.shape.features;
		}

		roi->add_tensor(std::make_shared<HailoTensor>(t.data.get(), info));
	}

	return roi;
}

Rectangle HailoPostProcessingStage::ConvertInferenceCoordinates(const std::vector<float> &coords,
																const std::vector<Rectangle> &scaler_crops) const
{
	if (coords.size() != 4 || scaler_crops.size() != 2)
		return {};

	// Convert the inference image co-ordinates into the final ISP output co-ordinates.
	const Size &isp_output_size = output_stream_->configuration().size;

	// Object scaled to the full sensor resolution
	Rectangle obj;
	obj.x = std::round(coords[0] * (scaler_crops[1].width - 1));
	obj.y = std::round(coords[1] * (scaler_crops[1].height - 1));
	obj.width = std::round(coords[2] * (scaler_crops[1].width - 1));
	obj.height = std::round(coords[3] * (scaler_crops[1].height - 1));

	// Object on the low res scaler crop -> translated by the start of the low res crop offset
	const Rectangle obj_translated_l = obj.translatedBy(scaler_crops[1].topLeft());
	// -> bounded to the high res output crop
	const Rectangle obj_bounded = obj_translated_l.boundedTo(scaler_crops[0]);
	// -> translated to the start of the high res crop offset
	const Rectangle obj_translated_h = obj_bounded.translatedBy(-scaler_crops[0].topLeft());
	// -> and scaled to the ISP output.
	const Rectangle obj_scaled = obj_translated_h.scaledBy(isp_output_size, scaler_crops[0].size());

	return obj_scaled;
}

// Mouse callback for exiting fullscreen
void mouseCallback(int event, int, int, int, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		Display::ToggleFullscreen();
	}
}

cv::Size getScreenDimensions()
{
	const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
	const char* x_display = std::getenv("DISPLAY");
	
	if (wayland_display || x_display)
	{
		std::ifstream fbinfo("/sys/class/graphics/fb0/virtual_size");
		if (fbinfo.is_open())
		{
			std::string line;
			if (std::getline(fbinfo, line))
			{
				std::istringstream iss(line);
				int width, height;
				char comma;
				if (iss >> width >> comma >> height)
				{	
					return {width, height};
				}
			}
		}
	}
	
	return {800, 600};
}

void Display::displayThread()
{
	RgbImagePtr current_image;

	while (true)
	{
		Msg msg = msg_queue_.Wait();

		if (msg.type == MsgType::Quit)
			break;

		if (msg.type == MsgType::Display)
		{
			current_image = std::move(msg.payload);

			// RGB -> BGR for cv::imshow
			for (unsigned int j = 0; j < msg.size.height; j++)
			{
				uint8_t *ptr = current_image.get() + j * msg.size.width * 3;
				for (unsigned int i = 0; i < msg.size.width; i++)
				{
					const uint8_t t = ptr[0];
					ptr[0] = ptr[2], ptr[2] = t;
					ptr += 3;
				}
			}
			
			cv::Mat image(msg.size.height, msg.size.width, CV_8UC3, (void *)current_image.get(),
						  msg.size.width * 3);

			cv::namedWindow(msg.window_title, cv::WINDOW_NORMAL);
			cv::setMouseCallback(msg.window_title, mouseCallback, nullptr);
			cv::Mat display_image = image;
			if (fullscreen_)
			{
				cv::setWindowProperty(msg.window_title, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

				auto screen = getScreenDimensions();
				if (screen.width > 0 && screen.height > 0)
				{
					cv::resize(image, display_image, screen, 0, 0, cv::INTER_LINEAR);
				}
			}
			else
			{
				cv::setWindowProperty(msg.window_title, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
				cv::resizeWindow(msg.window_title, cv::Size(window_width_, window_height_));
			}

			cv::imshow(msg.window_title, display_image);
			cv::waitKey(1);
		}
	}
}
