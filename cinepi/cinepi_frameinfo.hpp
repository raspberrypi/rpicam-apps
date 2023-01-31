#pragma once

#include <cstdio>
#include <string>

#include <linux/bcm2835-isp.h>
#include <libcamera/controls.h>
#include "core/frame_info.hpp"

#define HISTOGRAM_SIZE 3*NUM_HISTOGRAM_BINS

struct CinePIFrameInfo : public FrameInfo
{
	CinePIFrameInfo(libcamera::ControlList &ctrls)
		: FrameInfo(ctrls)
	{
		auto colorT = ctrls.get(libcamera::controls::ColourTemperature);
		if (colorT)
			colorTemp = *colorT;


        auto sts = ctrls.get(libcamera::controls::SensorTimestamp);
        if(sts){
            ts = (*sts);
        }


        #ifdef LIBCAMERA_CINEPI_CONTROLS 
		    auto histo = ctrls.get(libcamera::controls::RawHistogram);
            if(histo){
                memcpy(histogram,&(*histo)[0],sizeof(histogram));
            }
        #else
            for(int i = 0; i < (HISTOGRAM_SIZE); i++){
                histogram[i] = -1;
            }
        #endif
	}

    std::string histoString() const{
        std::ostringstream os;
        os << histogram[0] << "," << histogram[1*NUM_HISTOGRAM_BINS] << "," << histogram[2*NUM_HISTOGRAM_BINS] 
            << "," << histogram[((1*NUM_HISTOGRAM_BINS)-1)] << "," << histogram[((2*NUM_HISTOGRAM_BINS)-1)] << "," << histogram[((3*NUM_HISTOGRAM_BINS)-1)];
        return os.str();
    };

	unsigned int colorTemp;
    int32_t histogram[HISTOGRAM_SIZE];
    int64_t ts;
};