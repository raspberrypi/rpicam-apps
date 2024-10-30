// Copyright (c) Chris Hafey.
// SPDX-License-Identifier: MIT

#pragma once

struct Point {
    Point() : x(0), y(0) {}
    Point(uint32_t x, uint32_t y) : x(x), y(y) {}

    uint32_t x;
    uint32_t y;
};