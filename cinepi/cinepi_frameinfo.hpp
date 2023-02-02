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
		: FrameInfo(ctrls), threshold_l(25.0), threshold_h(2.0), trafficLight(0)
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

            auto histo_stats = ctrls.get(libcamera::controls::RawHistogramExt);
            if(histo_stats){
                histogram_stats[0] = (*histo_stats)[0];
                histogram_stats[1] = (*histo_stats)[1];
                histogram_stats[2] = (*histo_stats)[2];
                histogram_stats[3] = (*histo_stats)[3];
                histogram_stats[4] = (*histo_stats)[4];
                histogram_stats[5] = (*histo_stats)[5];
                histogram_stats[6] = (*histo_stats)[6];
                histogram_stats[7] = (*histo_stats)[7];
                histogram_stats[8] = (*histo_stats)[8];

                rL = ((float)histogram_stats[3] / (float)histogram_stats[0])*100.0;
                gL = ((float)histogram_stats[4] / (float)histogram_stats[1])*100.0;
                bL = ((float)histogram_stats[5] / (float)histogram_stats[2])*100.0;

                rH = ((float)histogram_stats[6] / (float)histogram_stats[0])*100.0;
                gH = ((float)histogram_stats[7] / (float)histogram_stats[1])*100.0;
                bH = ((float)histogram_stats[8] / (float)histogram_stats[2])*100.0;

                trafficLight = 0;
                trafficLight ^= (-(rL > threshold_l) ^ trafficLight) & (0x01);
                trafficLight ^= (-(gL > threshold_l) ^ trafficLight) & (0x02);
                trafficLight ^= (-(bL > threshold_l) ^ trafficLight) & (0x04);

                trafficLight ^= (-(rH > threshold_h) ^ trafficLight) & (0x10);
                trafficLight ^= (-(gH > threshold_h) ^ trafficLight) & (0x20);
                trafficLight ^= (-(bH > threshold_h) ^ trafficLight) & (0x40);
            }
        #else
            for(int i = 0; i < (HISTOGRAM_SIZE); i++){
                histogram[i] = -1;
            }
        #endif
	}

    std::string histoString() const{
        std::ostringstream os;
        os << rL << "%, " << gL << "%, " << bL << "% : " << rH << "%, " << gH << "%, " << bH << "%";
        // os << (unsigned int)trafficLight;
        return os.str();
    };

    float threshold_h;
    float threshold_l;
    uint8_t trafficLight;
    float rL, gL, bL, rH, gH, bH;

	unsigned int colorTemp;
    int32_t histogram[HISTOGRAM_SIZE];
    int32_t histogram_stats[9];
    unsigned int sums[3];
    int64_t ts;
};