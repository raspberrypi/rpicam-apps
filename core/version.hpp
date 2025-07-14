/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 */
#pragma once

extern "C"
{
	const char *RPiCamAppsVersion();
	const char *RPiCamAppsCapabilities(const std::string &preview_libs, const std::string &encoder_libs);
}
