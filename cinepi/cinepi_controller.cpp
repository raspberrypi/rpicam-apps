#include "cinepi_controller.hpp"

using namespace std;
using namespace std::chrono;


void CinePIController::sync(){
    auto pipe = redis_->pipeline();
    auto pipe_replies = pipe.get(CONTROL_KEY_ISO)
                            .exec();

    is_recording_ = (bool)stoi(*pipe_replies.get<OptionalString>(0));
}

void CinePIController::process(CompletedRequestPtr &completed_request){
    CinePIFrameInfo info(completed_request->metadata);

    redis_->publish(CHANNEL_STATS, to_string(completed_request->framerate));
    redis_->publish(CHANNEL_STATS, to_string(info.colorTemp));
    redis_->publish(CHANNEL_STATS, to_string(app_->GetEncoder()->getFrameCount()));
    redis_->publish(CHANNEL_STATS, to_string(app_->GetEncoder()->recording()));
}


void CinePIController::mainThread(){

    time_point<system_clock> t = system_clock::now();

    std::cout << "mainThread Started!" << std::endl;
    auto sub = redis_->subscriber();

    sub.on_message([this](std::string channel, std::string msg) {
        std::cout << msg << " from: " << channel << std::endl;
        auto r = redis_->get(msg);
        if(r){
            if(msg.compare(CONTROL_KEY_RECORD) == 0){
                // std::cout << *r << std::endl;
                trigger_ = !is_recording_;
                is_recording_ = (bool)stoi(*r);
            } 
            else if (msg.compare(CONTROL_KEY_ISO) == 0){
                iso_ = (unsigned int)(stoi(*r)/100.0);
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AnalogueGain, iso_);
                app_->SetControls(cl);
            }
            else if (msg.compare(CONTROL_KEY_WB) == 0){
                awb_ = (unsigned int)(stoi(*r));
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AwbEnable, awb_);
                app_->SetControls(cl);
            } 
            else if (msg.compare(CONTROL_KEY_COLORGAINS) == 0){
                std::string cg_rb_s = *r;
                char *ptr = strtok(&cg_rb_s[0], ",");
                uint8_t i = 0;
                while(ptr != NULL){
                    cg_rb_[i] = (float)stof(ptr);
                    i++;
                    ptr = strtok(NULL, ",");  
                }
                libcamera::ControlList cl;
                cl.set(libcamera::controls::ColourGains, libcamera::Span<const float, 2>({ cg_rb_[0], cg_rb_[1] }));
                app_->SetControls(cl);
            }
            else if (msg.compare(CONTROL_KEY_SHUTTER_ANGLE) == 0){
                // StreamInfo info = app_->GetStreamInfo(app->RawStream());
                shutter_angle_ = stof(*r);
                shutter_speed_ = 1.0 / ((framerate_ * 360.0) / shutter_angle_);
                uint64_t shutterTime = shutter_speed_ * 1e+6;
                libcamera::ControlList cl;
                cl.set(libcamera::controls::ExposureTime, shutterTime);
                app_->SetControls(cl);
            } 
            else if (msg.compare(CONTROL_KEY_SHUTTER_SPEED) == 0){
                shutter_speed_ = 1.0 / stof(*r);
                uint64_t shutterTime = shutter_speed_ * 1e+6;
                libcamera::ControlList cl;
                cl.set(libcamera::controls::ExposureTime, shutterTime);
                app_->SetControls(cl);
            }
        }
    });

    sub.subscribe(CHANNEL_CONTROLS);

    while (true) {
        try {
            sub.consume();
        } catch (const Error &err) {
            // Handle exceptions.
        }

        t += milliseconds(10);
        this_thread::sleep_until(t);
    }
}