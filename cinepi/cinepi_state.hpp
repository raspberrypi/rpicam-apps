#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdint.h>

#include "core/logging.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

#include <mutex>
#include <queue>
#include <thread>

#define CONTROL_KEY_RECORD "is_recording"
#define CONTROL_KEY_ISO "iso"
#define CONTROL_KEY_WB "awb"
#define CONTROL_KEY_COLORGAINS "cg_rb"
#define CONTROL_KEY_SHUTTER_ANGLE "shutter_a"
#define CONTROL_KEY_SHUTTER_SPEED "shutter_s"

class CinePIState
{
    public:
        CinePIState() : is_recording_(false) {};
        ~CinePIState() {};

        bool isRecording(){
            return is_recording_;
        }

        unsigned int getClipNumber(){
            return clip_number_;
        }

    protected:
        float framerate_;
        bool is_recording_;
        unsigned int iso_;
        unsigned int awb_;
        float shutter_speed_;
        float shutter_angle_;
        unsigned int colot_temp_;
        float cg_rb_[2];

        unsigned int clip_number_;
        
};