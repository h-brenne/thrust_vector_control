#include <stdexcept>
#include <iostream>
#include "thrust_vector_controller.h"
//using namespace mjbots;
//using MoteusInterface = moteus::Pi3HatMoteusInterface;

ThrustVectorController::ThrustVectorController(){};

void ThrustVectorController::initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    stop_ = false;
    cycle_count_ = 0;
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt8;
    res.velocity = moteus::Resolution::kInt16;
    res.feedforward_torque = moteus::Resolution::kIgnore;
    res.kp_scale = moteus::Resolution::kIgnore;
    res.kd_scale = moteus::Resolution::kIgnore;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    for (auto& cmd : *commands) {
    cmd.resolution = res;
    }
    start_time_ = std::chrono::steady_clock::now();
}

moteus::QueryResult ThrustVectorController::get(const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) {
    for (const auto& item : replies) {
        if (item.id == id && item.bus == bus) { return item.result; }
    }
    return {};
}

bool ThrustVectorController::run(const std::vector<MoteusInterface::ServoReply>& status,
        std::vector<MoteusInterface::ServoCommand>* output) {
    bool stop = false;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;
    if (elapsed.count() < 0.1 or elapsed.count() > 5.0) {
        for (auto& cmd : *output) {
            // We start everything with a stopped command to clear faults.
            cmd.mode = moteus::Mode::kStopped;
        }
    } else {
        auto& first_out = output->at(0);  // We constructed this, so we know the order.
        first_out.mode = moteus::Mode::kSinusoidal;
        first_out.position.position = std::numeric_limits<double>::quiet_NaN();
        first_out.position.velocity = 4;
        first_out.position.sinusoidal_amplitude = 0.8;
        first_out.position.sinusoidal_phase = 0.0;
    }
    if (elapsed.count() > 5.0) {
        stop = true;
    } else { 
        stop = false;
    }
    return stop;
    
}