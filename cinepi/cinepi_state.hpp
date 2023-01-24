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

class CinePIState
{
    public:
        CinePIState() : is_recording_(false) {};
        ~CinePIState() {};

        bool isRecording(){
            return is_recording_;
        }

    protected:
        bool is_recording_;
        unsigned int iso_;
        
};