/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 * Copyright (C) 2025, github.com/Kletternaut
 *
 * acoustic_focus_stage.cpp
 * Version 0.1
 *
 * acoustic_focus_stage.cpp - Stage to access libcamera FoM (Focus Figure of Merit)
 * on an acoustic way. The AcousticFocusStage is a post-processing stage for rpicam-apps
 * that outputs a sine tone whose frequency depends on FoM value.
 * This allows you to find the optimal focus point of a manual lens
 * **without needing to look at or interpret the preview image**.
 * Sound hardware is required to hear the output.
 */

#include <libcamera/stream.h>
#include <libcamera/controls.h>
#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <thread>

using Stream = libcamera::Stream;

class AcousticFocusStage : public PostProcessingStage
{
public:
    AcousticFocusStage(RPiCamApp *app)
        : PostProcessingStage(app)
    {}

    char const *Name() const override { return "acoustic_focus"; }

    void Configure() override
    {
        // Optional: Streams initialisieren, falls benötigt
        stream_ = app_->GetStream("video");
        if (!stream_)
            throw std::runtime_error("AcousticFocusStage: Stream 'video' not found!");
    }

    bool Process(CompletedRequestPtr &completed_request) override
    {
        static auto last = std::chrono::steady_clock::now();

        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();

        if (ms >= 1000) // nur alle 1000 ms (1 Sekunde)
        {
            last = now;

            auto fom = completed_request->metadata.get(libcamera::controls::FocusFoM);
            if (fom)
            {
                // Mapping: FoM 0..1000 → 400..2000 Hz (Beispielwerte)
                int min_fom = 0, max_fom = 1000;
                int min_freq = 400, max_freq = 2000;
                int freq = min_freq;
                if (*fom > min_fom)
                    freq = min_fom + ((*fom - min_fom) * (max_freq - min_freq)) / (max_fom - min_fom);
                freq = std::min(max_freq, std::max(min_freq, freq));

                double duration = 0.1; // 0.1 Sekunden
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(6) << duration;
                std::string duration_str = oss.str();

                std::string cmd = "/usr/bin/play -nq -t alsa synth " + duration_str +
                                  " sine " + std::to_string(freq);
                std::thread([](std::string cmd) { system(cmd.c_str()); }, cmd).detach();
            }
        }
        return false;
    }

private:
    Stream *stream_ = nullptr;
};

// Registrierung der Stage im Framework
static PostProcessingStage *Create(RPiCamApp *app)
{
    return new AcousticFocusStage(app);
}
static RegisterStage reg("acoustic_focus", &Create);