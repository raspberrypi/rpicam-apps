/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_still.cpp - libcamera stills capture app.
 */
#include <chrono>
#include <filesystem>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include <chrono>

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"

#include "output/output.hpp"

#include "image/image.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using libcamera::Stream;

namespace fs = std::filesystem;

class RPiCamStillApp : public RPiCamApp
{
public:
	RPiCamStillApp() : RPiCamApp(std::make_unique<StillOptions>()) {}

	StillOptions *GetOptions() const { return static_cast<StillOptions *>(options_.get()); }
};

/////////////////////////////////////////////////////////////////////
#include <iostream>
#include <opencv2/opencv.hpp> // OpenCV 라이브러리 포함
using libcamera::Stream;

// OpenCV에서 사용할 Mat로 변환하는 함수
cv::Mat libcamera_to_mat(const std::vector<libcamera::Span<uint8_t>> &mem, int width, int height)
{
	// libcamera의 이미지 데이터를 OpenCV Mat로 변환
	// 만약 데이터를 RGB 또는 YUV 포맷으로 받았다면 그에 맞게 변환 필요
	std::cout << "hello...?" << std::endl;
	std::cout << mem.size() << std::endl;

	return cv::Mat(height, width, CV_8UC3, (void *)mem[0].data());
}

// 이미지를 캡처하고 OpenCV Mat으로 변환하는 함수
static void save_image_as_mat(RPiCamStillApp &app, CompletedRequestPtr &payload, Stream *stream)
{
	StreamInfo info = app.GetStreamInfo(stream);
	BufferReadSync r(&app, payload->buffers[stream]);
	const std::vector<libcamera::Span<uint8_t>> mem = r.Get();

	// 이미지를 cv::Mat으로 변환
	cv::Mat img = libcamera_to_mat(mem, info.width, info.height);
	std::cout << "HIHIHI" << std::endl;
	std::cout << info.pixel_format << std::endl << libcamera::formats::RGB888 << std::endl;
	// 이미지의 크기 출력
	std::cout << "Rows: " << img.rows << ", Cols: " << img.cols << std::endl;

	// 채널 수 출력 (1: 그레이스케일, 3: RGB 등)
	std::cout << "Channels: " << img.channels() << std::endl;

	// 데이터 타입 확인
	// std::cout << "Type: " << img.type() << std::endl;

	// cv::imwrite("output_image.jpg", img); // 변환된 이미지를 파일로 저장

	// OpenCV의 다양한 함수를 사용해 이미지 처리 가능
	cv::imshow("Captured Image", img); // 이미지 표시
	std::cout << "HIHIHI" << std::endl;
	cv::waitKey(30); // 키 입력을 기다림
	// std::cout << "HIHIHI" << std::endl;
}

// main 루프 내에서 호출하는 부분
static void event_loop(RPiCamStillApp &app)
{
	StillOptions const *options = app.GetOptions();
	std::cout << "ENCODING: " << options->encoding << std::endl;
	unsigned int still_flags = RPiCamApp::FLAG_STILL_NONE;
	still_flags |= RPiCamApp::FLAG_STILL_RGB;

	// Options const *options = app.GetOptions();
	app.OpenCamera();
	app.ConfigureStill(still_flags);
	// app.ConfigureViewfinder();
	app.StartCamera();

	for (unsigned int count = 0;; count++)
	{
		RPiCamApp::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::RequestComplete)
		{
			CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
			// app.ShowPreview(completed_request, app.ViewfinderStream());
			// OpenCV Mat으로 변환하고 처리
			save_image_as_mat(app, completed_request, app.StillStream());
		}
		// 나머지 처리...
	}
}

/////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	try
	{
		RPiCamStillApp app;
		StillOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
