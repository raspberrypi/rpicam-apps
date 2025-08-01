/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_detect_draw_cv_stage.cpp - draw object detection results
 */

#include "opencv2/imgproc.hpp"

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "object_detect.hpp"

#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

using namespace cv;

using Rectange = libcamera::Rectangle;
using Stream = libcamera::Stream;

// Define the start delimiter for our binary protocol
constexpr static uint32_t START_DELIMITER = 0xDDCCBBAA; // little-endian representation of 0xAA, 0xBB, 0xCC, 0xDD
#define UDP_IP_DEFAULT false
#define UDP_IP "127.0.0.1" 
#define UDP_PORT 12347

class ObjectDetectDrawCvStage : public PostProcessingStage
{
public:
	ObjectDetectDrawCvStage(RPiCamApp *app) : PostProcessingStage(app), sockfd_(-1) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;
	
	bool Process(CompletedRequestPtr &completed_request) override;

	virtual ~ObjectDetectDrawCvStage() override;
	
private:
	Stream *stream_;
	int line_thickness_;
	double font_size_;
	
	bool udp_broadcast_enabled;
	std::string udp_broadcast_address;
	u_int16_t udp_broadcast_port;
	int sockfd_;
    struct sockaddr_in servaddr_;
};

#define NAME "object_detect_draw_cv"

char const *ObjectDetectDrawCvStage::Name() const
{
	return NAME;
}


ObjectDetectDrawCvStage::~ObjectDetectDrawCvStage()
{
    if (sockfd_ != -1)
    {
        close(sockfd_);
        std::cerr << "UDP socket closed." << std::endl;
    }
}


void ObjectDetectDrawCvStage::Configure()
{
	stream_ = app_->GetMainStream();
	
	// Initialize UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        perror("UDP socket creation failed");
        // Handle error, perhaps throw an exception or exit
        return;
    }

    memset(&servaddr_, 0, sizeof(servaddr_));

    // Filling server information
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(udp_broadcast_port);
    if (inet_pton(AF_INET, udp_broadcast_address.c_str(), &servaddr_.sin_addr) <= 0)
    {
        perror("Invalid address/ Address not supported");
        close(sockfd_);
        sockfd_ = -1; // Mark as invalid
        return;
    }

    std::cerr << "UDP socket initialized for IP: " << udp_broadcast_address << ", Port: " << udp_broadcast_port << std::endl;
}


void ObjectDetectDrawCvStage::Read(boost::property_tree::ptree const &params)
{
	line_thickness_ = params.get<int>("line_thickness", 1);
	font_size_ = params.get<double>("font_size", 1.0);
	udp_broadcast_enabled = params.get<bool>("enable_ip", UDP_IP_DEFAULT);
	udp_broadcast_address = params.get<std::string>("ip", UDP_IP);
	udp_broadcast_port = params.get<u_int16_t>("port", UDP_PORT);
}

bool ObjectDetectDrawCvStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *ptr = (uint32_t *)buffer.data();
	StreamInfo info = app_->GetStreamInfo(stream_);

	std::vector<Detection> detections;

	completed_request->post_process_metadata.Get("object_detect.results", detections);

	Mat image(info.height, info.width, CV_8U, ptr, info.stride);
	Scalar colour = Scalar(255, 255, 255);
	int font = FONT_HERSHEY_SIMPLEX;

	for (auto &detection : detections)
	{
		// Draw rectangle and text on the image
		Rect r(detection.box.x, detection.box.y, detection.box.width, detection.box.height);
		rectangle(image, r, colour, line_thickness_);
		std::stringstream text_stream;
		text_stream << detection.name << " " << (int)(detection.confidence * 100) << "%";
		std::string text = text_stream.str();
		int baseline = 0;
		Size size = getTextSize(text, font, font_size_, 2, &baseline);
		Point text_origin(detection.box.x + 5, detection.box.y + size.height + 5);
		putText(image, text, text_origin, font, font_size_, colour, 2);

        // Prepare and send data for UDP transmission using the new binary protocol
        if ( udp_broadcast_enabled && (sockfd_ != -1)) // Only send if socket is valid
        {
            // Use a vector to dynamically build the binary message
            std::vector<char> udp_data_buffer;

            // 1. Add start delimiter (4 bytes)
            udp_data_buffer.insert(udp_data_buffer.end(), (char*)&START_DELIMITER, (char*)&START_DELIMITER + sizeof(START_DELIMITER));

            // 2. Add x, y, width, height (4 bytes each)
            const int32_t x = detection.box.x;
            const int32_t y = detection.box.y;
            const int32_t width = detection.box.width;
            const int32_t height = detection.box.height;
            udp_data_buffer.insert(udp_data_buffer.end(), (char*)&x, (char*)&x + sizeof(x));
            udp_data_buffer.insert(udp_data_buffer.end(), (char*)&y, (char*)&y + sizeof(y));
            udp_data_buffer.insert(udp_data_buffer.end(), (char*)&width, (char*)&width + sizeof(width));
            udp_data_buffer.insert(udp_data_buffer.end(), (char*)&height, (char*)&height + sizeof(height));

            // 3. Add name length and name string
            uint8_t name_length = detection.name.length();
            if (name_length > 255) {
                // Truncate or handle error for names longer than 255 chars
                name_length = 255;
            }
            udp_data_buffer.push_back(name_length);
            udp_data_buffer.insert(udp_data_buffer.end(), detection.name.begin(), detection.name.begin() + name_length);

            // 4. Add confidence (4 bytes)
            const float confidence = detection.confidence;
            udp_data_buffer.insert(udp_data_buffer.end(), (char*)&confidence, (char*)&confidence + sizeof(confidence));

            // Send data via UDP
            const ssize_t bytes_sent = sendto(sockfd_, udp_data_buffer.data(), udp_data_buffer.size(), 0,
                                        (const struct sockaddr *)&servaddr_, sizeof(servaddr_));
            if (bytes_sent < 0)
            {
                perror("Failed to send UDP message");
            }
        }
	}

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectDetectDrawCvStage(app);
}

static RegisterStage reg(NAME, &Create);
