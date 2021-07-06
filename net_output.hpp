/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * net_output.hpp - send output over network.
 */

#pragma once

#include <netinet/in.h>

#include "output.hpp"

class NetOutput : public Output
{
public:
	NetOutput(VideoOptions const *options);
	~NetOutput();

protected:
	void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;

private:
	int fd_;
	sockaddr_in saddr_;
	const sockaddr *saddr_ptr_;
	socklen_t sockaddr_in_size_;
};
