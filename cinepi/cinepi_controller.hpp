#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdint.h>

#include "dng_encoder.hpp"
#include "preview/preview.hpp"
#include "core/logging.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

#include <mutex>
#include <queue>
#include <thread>

#include "utils.hpp"

#include "cinepi_frameinfo.hpp"
#include "core/stream_info.hpp"

#include "cinepi_recorder.hpp"
#include "cinepi_state.hpp"
#include "raw_options.hpp"
#include <sw/redis++/redis++.h>

#define CHANNEL_CONTROLS "cp_controls"
#define CHANNEL_STATS "cp_stats"

using namespace sw::redis;

class CinePIController : public CinePIState
{
    public:
        CinePIController(CinePIRecorder *app) : CinePIState(), app_(app), options_(app->GetOptions()), folderOpen(false), abortThread_(false) {};
        ~CinePIController() {
            abortThread_ = true;
            main_thread_.join();
        };

        void start(){
            redis_ = new Redis(options_->redis);
            std::cout << redis_->ping() << std::endl;
            // sync();
            main_thread_ = std::thread(std::bind(&CinePIController::mainThread, this));
            // pub_thread_ = std::thread(std::bind(&CinePIController::pubThread, this));
        }

        void sync();

        void process(CompletedRequestPtr &completed_request);

        bool folderOpen;

        int triggerRec(){
            if(!disk_mounted()){
                return 0;
            }
            int state = trigger_;
            if(state < 0){
                clip_number_++;
            }
            trigger_ = 0;
            return state;
        }

    protected:

    private:
        void mainThread();
        void pubThread();

        int trigger_;

        CinePIRecorder *app_;
        RawOptions *options_;
        Redis *redis_;

        bool abortThread_;
        std::thread main_thread_;
        std::thread pub_thread_;
};