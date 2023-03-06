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
    res.sinusoidal_amplitude = moteus::Resolution::kInt16;
    res.sinusoidal_phase = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kIgnore;
    res.kd_scale = moteus::Resolution::kIgnore;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    
    moteus::QueryCommand query_cmd;
    // We don't care about position
    query_cmd.position = moteus::Resolution::kIgnore;
    query_cmd.control_velocity = moteus::Resolution::kInt16;

    for (auto& cmd : *commands) {
        cmd.resolution = res;
        cmd.query = query_cmd;
    }
    start_time_ = std::chrono::steady_clock::now();
}

moteus::QueryResult ThrustVectorController::get(const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) {
    for (const auto& item : replies) {
        if (item.id == id && item.bus == bus) { return item.result; }
    }
    return {};
}

float ThrustVectorController::value_sweep(float start_value, float end_value, float elapsed_seconds, float end_time_seconds) {
    return start_value+(elapsed_seconds/end_time_seconds)*(end_value-start_value);
}

void ThrustVectorController::startup_sequence(MoteusInterface::ServoCommand* command, float elapsed_seconds, float end_time_seconds) {
    float end_velocity = 5.0;
    command->position.velocity = value_sweep(0,end_velocity, elapsed_seconds, end_time_seconds);
    return;
}

bool ThrustVectorController::run(const std::vector<MoteusInterface::ServoReply>& status,
        std::vector<MoteusInterface::ServoCommand>* output) {
    bool stop = false;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;
    
    auto& first_out = output->at(0);  // We constructed this, so we know the order.
    // Startup sequence
    // Makes sure that the hinged rotor folds out more gracefully
    float startup_sequence_length_seconds = 1.0;
    float startup_sequence_end_velocity = 10;
    if (elapsed.count() < startup_sequence_length_seconds) {
        first_out.mode = moteus::Mode::kSinusoidal;
        startup_sequence(&first_out, elapsed.count(), startup_sequence_length_seconds);
    } else {
        first_out.mode = moteus::Mode::kSinusoidal;
        first_out.position.position = std::numeric_limits<double>::quiet_NaN();
        first_out.position.velocity = 10;
        first_out.position.sinusoidal_amplitude = 0.8;
        first_out.position.sinusoidal_phase = 0.0;
    }
    if (elapsed.count() > 5.0) {
        first_out.mode = moteus::Mode::kStopped;
        stop = true;
    } else { 
        stop = false;
    }
    return stop;
}