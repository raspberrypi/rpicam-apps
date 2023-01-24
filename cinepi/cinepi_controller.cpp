#include "cinepi_controller.hpp"

using namespace std;
using namespace std::chrono;


void CinePIController::sync(){
    auto pipe = redis_->pipeline();
    auto pipe_replies = pipe.get(CONTROL_KEY_ISO)
                            .exec();

    is_recording_ = (bool)stoi(*pipe_replies.get<OptionalString>(0));
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
                is_recording_ = (bool)stoi(*r);
            } 
            else if (msg.compare(CONTROL_KEY_ISO) == 0){
                std::cout << *r << std::endl;
                iso_ = (unsigned int)stoi(*r);
                libcamera::ControlList cl;
                cl.set(libcamera::controls::AnalogueGain, iso_);
                app_->SetControls(cl);
            }
        }
    });

    sub.subscribe(CHANNEL_CONTROLS);

    // Consume messages in a loop.
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