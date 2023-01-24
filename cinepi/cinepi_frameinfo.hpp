#pragma once

#include <cstdio>
#include <string>

#include "core/frame_info.hpp"

struct CinePIFrameInfo : public FrameInfo
{
	CinePIFrameInfo(libcamera::ControlList &ctrls)
		: FrameInfo(ctrls)
	{
		auto colorT = ctrls.get(libcamera::controls::ColourTemperature);
		if (colorT)
			colorTemp = *colorT;

        #if LIBCAMERA_CINEPI
		    auto histo = ctrls.get(libcamera::controls::draft::SensorRollingShutterSkew);
            if(histo){
                histogram[0] = (int32_t)(*histo)[0]
                histogram[1] = (int32_t)(*histo)[1]
                histogram[2] = (int32_t)(*histo)[2]
                histogram[3] = (int32_t)(*histo)[3]
                histogram[4] = (int32_t)(*histo)[4]
                histogram[5] = (int32_t)(*histo)[5]
            }
        #endif
	}


	unsigned int colorTemp;
    int32_t histogram[6];
};