/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Limited
 *
 * object_detect_udp.cpp - sends detection results over UDP socket
 */

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "object_detect.hpp"

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>


using Rectange = libcamera::Rectangle;
using Stream = libcamera::Stream;

// Define the start delimiter for our binary protocol
constexpr static uint32_t START_DELIMITER = 0xDDCCBBAA; // little-endian representation of 0xAA, 0xBB, 0xCC, 0xDD
#define UDP_IP_DEFAULT false
#define UDP_IP "127.0.0.1"
#define UDP_PORT 12347

class ObjectDetectUDPStage : public PostProcessingStage
{
public:
	ObjectDetectUDPStage(RPiCamApp *app) : PostProcessingStage(app), sockfd_(-1) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;
	
	bool Process(CompletedRequestPtr &completed_request) override;

	virtual ~ObjectDetectUDPStage() override;
	
private:
	Stream *stream_;
	std::string udp_broadcast_address = "127.0.0.1";
	u_int16_t udp_broadcast_port = 12345;
	int sockfd_;
	struct sockaddr_in servaddr_;
};

#define NAME "object_detect_udp"

char const *ObjectDetectUDPStage::Name() const
{
	return NAME;
}


ObjectDetectUDPStage::~ObjectDetectUDPStage()
{
	if (sockfd_ != -1)
	{
		close(sockfd_);
		std::cerr << "UDP socket closed." << std::endl;
	}
}


void ObjectDetectUDPStage::Configure()
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

	std::cerr
		<< "UDP socket initialized for IP: " << udp_broadcast_address << ", Port: " << udp_broadcast_port << std::endl;
}

void ObjectDetectUDPStage::Read(boost::property_tree::ptree const &params)
{
	udp_broadcast_address = params.get<std::string>("ip", UDP_IP);
	udp_broadcast_port = params.get<u_int16_t>("port", UDP_PORT);
}

template <typename T>
void append(std::vector<char> &buffer, const T &value)
{
	const char *raw = reinterpret_cast<const char *>(&value);
	buffer.insert(buffer.end(), raw, raw + sizeof(T));
}

bool ObjectDetectUDPStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	std::vector<Detection> detections;
	completed_request->post_process_metadata.Get("object_detect.results", detections);

	if (sockfd_ == -1)
		return false;

	for (auto &detection : detections)
	{
		// Draw rectangle and text on the image
		std::stringstream text_stream;
		text_stream << detection.name << " " << (int)(detection.confidence * 100) << "%";
		std::string text = text_stream.str();
		
		// Use a vector to dynamically build the binary message
		std::vector<char> udp_data_buffer;

		// 1. Add start delimiter (4 bytes)
		append(udp_data_buffer, START_DELIMITER);

		// 2. Add x, y, width, height (4 bytes each)
		const int32_t x = detection.box.x;
		const int32_t y = detection.box.y;
		const int32_t width = detection.box.width;
		const int32_t height = detection.box.height;

		append(udp_data_buffer, x);
		append(udp_data_buffer, y);
		append(udp_data_buffer, width);
		append(udp_data_buffer, height);

		// 3. Add name length and name string
		constexpr uint8_t name_length = 255;
		udp_data_buffer.push_back(static_cast<char>(name_length));

		uint8_t name[name_length];
		memcpy(name, detection.name.c_str(), name_length - 2);
		name[name_length - 1] = '\0';
		append(udp_data_buffer, name);

		// 4. Add confidence (4 bytes)
		const float confidence = detection.confidence;
		append(udp_data_buffer, confidence);

		// Send data via UDP
		const ssize_t bytes_sent = sendto(sockfd_, udp_data_buffer.data(), udp_data_buffer.size(), 0,
										  (const struct sockaddr *)&servaddr_, sizeof(servaddr_));
		if (bytes_sent < 0)
			perror("Failed to send UDP message");
	}

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectDetectUDPStage(app);
}

static RegisterStage reg(NAME, &Create);
