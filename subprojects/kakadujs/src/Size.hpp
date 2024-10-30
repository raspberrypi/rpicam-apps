// Copyright (c) Chris Hafey.
// SPDX-License-Identifier: MIT

#pragma once

struct Size {
    Size() : width(0), height(0) {}
    Size(uint32_t width, uint32_t height) : width(width), height(height) {}

    uint32_t width;
    uint32_t height;
};