// Copyright (c) Chris Hafey.
// SPDX-License-Identifier: MIT

#pragma once

struct FrameInfo {
    /// <summary>
    /// Width of the image, range [1, 65535].
    /// </summary>
    uint16_t width;

    /// <summary>
    /// Height of the image, range [1, 65535].
    /// </summary>
    uint16_t height;

    /// <summary>
    /// Number of bits per sample, range [2, 16]
    /// </summary>
    uint8_t bitsPerSample;

    /// <summary>
    /// Number of components contained in the frame, range [1, 255]
    /// </summary>
    uint8_t componentCount;

    /// <summary>
    /// true if signed, false if unsigned
    /// </summary>
    bool isSigned;
};