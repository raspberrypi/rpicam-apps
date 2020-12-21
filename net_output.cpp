/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * net_output.cpp - send output over network.
 */

#include <sys/socket.h> 
#include <arpa/inet.h> 

#include "net_output.hpp"

NetOutput::NetOutput(VideoOptions const &options) : Output(options)
{
	char protocol[4];
	int start, end, a, b, c, d, port;
	if (sscanf(options.output.c_str(), "%3s://%n%d.%d.%d.%d%n:%d",
			   protocol, &start, &a, &b, &c, &d, &end, &port) != 6)
		throw std::runtime_error("bad network address " + options.output);
	std::string address = options.output.substr(start, end - start);

	if (strcmp(protocol, "udp") == 0)
	{
		saddr_ = {};
		saddr_.sin_family = AF_INET;
		saddr_.sin_port = htons(port);
		if (inet_aton(address.c_str(), &saddr_.sin_addr) == 0)
			throw std::runtime_error("inet_aton failed for " + address);

		fd_ = socket(AF_INET, SOCK_DGRAM, 0);
		if (fd_ < 0)
			throw std::runtime_error("unable to open udp socket");

		saddr_ptr_ = (const sockaddr *)&saddr_; // sendto needs these for udp
		sockaddr_in_size_ = sizeof(sockaddr_in);
	}
	else if (strcmp(protocol, "tcp") == 0)
	{
		// WARNING: I've not actually tried this yet...
		if (options.listen)
		{
			// We are the server.
			int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
			if (listen_fd < 0)
				throw std::runtime_error("unable to open listen socket");

			sockaddr_in server_saddr = {};
			server_saddr.sin_family = AF_INET;
			server_saddr.sin_addr.s_addr = INADDR_ANY;
			server_saddr.sin_port = htons(port);

			if (bind(listen_fd, (struct sockaddr *)&server_saddr, sizeof(server_saddr)) < 0)
				throw std::runtime_error("failed to bind listen socket");
			listen(listen_fd, 1);

			if (options.verbose)
				std::cout << "Waiting for client to connect..." << std::endl;
			fd_ = accept(listen_fd, (struct sockaddr *)&saddr_, &sockaddr_in_size_);
			if (fd_ < 0)
				throw std::runtime_error("accept socket failed");
			if (options.verbose)
				std::cout << "Client connection accepted" << std::endl;

			close(listen_fd);
		}
		else
		{
			// We are a client.
			saddr_ = {};
			saddr_.sin_family = AF_INET;
			saddr_.sin_port = htons(port);
			if (inet_aton(address.c_str(), &saddr_.sin_addr) == 0)
				throw std::runtime_error("inet_aton failed for " + address);

			fd_ = socket(AF_INET, SOCK_STREAM, 0);
			if (fd_ < 0)
				throw std::runtime_error("unable to open client socket");

			if (options.verbose)
				std::cout << "Connecting to server..." << std::endl;
			if (connect(fd_, (struct sockaddr *)&saddr_ , sizeof(sockaddr_in)) < 0)
				throw std::runtime_error("connect to server failed");
			if (options.verbose)
				std::cout << "Connected" << std::endl;
		}

		saddr_ptr_ = NULL; // sendto doesn't want these for tcp
		sockaddr_in_size_ = 0;
	}
	else
		throw std::runtime_error("unrecognised network protocol " + options.output);
}

NetOutput::~NetOutput()
{
	close(fd_);
}

void NetOutput::outputBuffer(void *mem, size_t size, int64_t /*timestamp_us*/, uint32_t /*flags*/)
{
	if (options_.verbose)
		std::cout << "NetOutput: output buffer " << mem << " size " << size << "\n";
	if (sendto(fd_, mem, size, 0, saddr_ptr_, sockaddr_in_size_) < 0)
		throw std::runtime_error("failed to send data on socket");
}
