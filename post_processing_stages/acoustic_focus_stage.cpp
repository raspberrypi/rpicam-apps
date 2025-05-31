/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Kletternaut
 *
 * acoustic_focus_stage.cpp - acoustic feedback for autofocus FoM
 *
 * This stage provides acoustic feedback based on the libcamera Autofocus Figure of Merit (FoM).
 * The FoM is mapped to an audible frequency, allowing users to hear focus quality changes in real time.
 * No visual contact with the preview is required, and the preview does not need to be interpreted.
 * As the FoM rises or falls, the tone frequency also rises or falls accordingly.
 * Various parameters (e.g. frequency range, mapping type, duration) can be configured via JSON.
 * Note: Sound output hardware must be present for this stage to function.
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
#include <cmath>
#include <fstream>


using Stream = libcamera::Stream;

class AcousticFocusStage : public PostProcessingStage
{
public:
    AcousticFocusStage(RPiCamApp *app) : PostProcessingStage(app)
    {
        boost::property_tree::ptree config;
        boost::property_tree::read_json("assets/acoustic_focus.json", config);
        if (auto focus = config.get_child_optional("acoustic_focus")) {
            Read(focus->front().second);
        }
    }

    char const *Name() const override { return "acoustic_focus"; }

    void Read(boost::property_tree::ptree const &params) override
    {
        min_fom_ = params.get<int>("minFoM", 1);
        max_fom_ = params.get<int>("maxFoM", 3000);
        min_freq_ = params.get<int>("minFreq", 300);
        max_freq_ = params.get<int>("maxFreq", 3000);
        duration_ = params.get<double>("duration", 0.1);
        mapping_ = params.get<std::string>("mapping", "log");
    }

    void Configure() override
    {
        stream_ = app_->GetStream("video");
        if (!stream_)
            throw std::runtime_error("AcousticFocusStage: Stream 'video' not found!");
    }

    bool Process(CompletedRequestPtr &completed_request) override
    {
        static auto last = std::chrono::steady_clock::now();

        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();

        if (ms >= 1000)
        {
            last = now;

            auto fom = completed_request->metadata.get(libcamera::controls::FocusFoM);
            if (fom)
            {
                int freq = min_freq_;
                if (mapping_ == "log") {
                    double norm = std::log(std::max(*fom, min_fom_)) - std::log(min_fom_);
                    double denom = std::log(max_fom_) - std::log(min_fom_);
                    freq = min_freq_ + static_cast<int>(norm / denom * (max_freq_ - min_freq_));
                } else { // linear
                    double norm = std::max(*fom, min_fom_) - min_fom_;
                    double denom = max_fom_ - min_fom_;
                    freq = min_freq_ + static_cast<int>(norm / denom * (max_freq_ - min_freq_));
                }
                freq = std::min(max_freq_, std::max(min_freq_, freq));

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(6) << duration_;
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
    int min_fom_ = 1, max_fom_ = 2000;
    int min_freq_ = 400, max_freq_ = 2000;
    double duration_ = 0.1;
    std::string mapping_ = "log";
};

static PostProcessingStage *Create(RPiCamApp *app)
{
    return new AcousticFocusStage(app);
}
static RegisterStage reg("acoustic_focus", &Create);