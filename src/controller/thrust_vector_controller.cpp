#include <stdexcept>
#include <iostream>
#include "thrust_vector_controller.h"
//using namespace mjbots;
//using MoteusInterface = moteus::Pi3HatMoteusInterface;

ThrustVectorController::ThrustVectorController(float velocity, float amplitude, float phase, float experiment_length, float startup_sequence_length)
    : velocity_(velocity)
    , amplitude_(amplitude)
    , phase_(phase)
    , experiment_length_(experiment_length)
    , startup_sequence_length_(startup_sequence_length)
{};

void ThrustVectorController::initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    cycle_count_ = 0;
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt8;
    res.velocity = moteus::Resolution::kFloat;
    res.feedforward_torque = moteus::Resolution::kIgnore;
    res.sinusoidal_amplitude = moteus::Resolution::kFloat;
    res.sinusoidal_phase = moteus::Resolution::kFloat;
    res.kp_scale = moteus::Resolution::kIgnore;
    res.kd_scale = moteus::Resolution::kIgnore;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;

    moteus::QueryCommand query_cmd;
    query_cmd.mode = moteus::Resolution::kInt16;
    query_cmd.position = moteus::Resolution::kIgnore;
    query_cmd.velocity = moteus::Resolution::kFloat;
    query_cmd.torque = moteus::Resolution::kFloat;
    query_cmd.q_current = moteus::Resolution::kIgnore;
    query_cmd.d_current = moteus::Resolution::kIgnore;
    query_cmd.rezero_state = moteus::Resolution::kInt8;
    query_cmd.voltage = moteus::Resolution::kInt8;
    query_cmd.temperature = moteus::Resolution::kInt8;
    query_cmd.fault = moteus::Resolution::kInt8;
    query_cmd.control_velocity = moteus::Resolution::kFloat;

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

void ThrustVectorController::constant_command_run(MoteusInterface::ServoCommand* command, float elapsed_seconds) {
    command->mode = moteus::Mode::kSinusoidal;
    command->position.position = std::numeric_limits<double>::quiet_NaN();
    command->position.maximum_torque = 0.1;
    command->position.velocity = velocity_;
    command->position.sinusoidal_amplitude = amplitude_;
    command->position.sinusoidal_phase = phase_;
    
    // New commands to fix bug
    command->position.kd_scale = 1.0;
    command->position.kp_scale = 1.0;
    command->position.feedforward_torque = 0;
    command->position.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    command->position.stop_position = std::numeric_limits<double>::quiet_NaN();
    
    bool is_finished = elapsed_seconds >= experiment_length_;
    return;
}


void ThrustVectorController::startup_sequence_run(MoteusInterface::ServoCommand* command, float elapsed_seconds) {
    command->mode = moteus::Mode::kSinusoidal;
    float end_velocity = velocity_;
    command->position.maximum_torque = 0.1;
    command->position.sinusoidal_amplitude = 0.0;
    command->position.sinusoidal_phase = 0.0;
    command->position.velocity = value_sweep(0,end_velocity, elapsed_seconds, startup_sequence_length_); 
    
    // New commands to fix bug
    command->position.kd_scale = 1.0;
    command->position.kp_scale = 1.0;
    command->position.feedforward_torque = 0;
    command->position.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    command->position.stop_position = std::numeric_limits<double>::quiet_NaN();

    
    return;
}

bool ThrustVectorController::run(const std::vector<MoteusInterface::ServoReply>& status,
        std::vector<MoteusInterface::ServoCommand>* output) {
    bool stop = false;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;
    
    // Only one motor controlled as of now
    auto& first_out = output->at(0);  // We constructed this, so we know the order.
    

    if (elapsed.count() < startup_sequence_length_) {
        // Startup sequence
        // Makes sure that the hinged rotor folds out more gracefully
        startup_sequence_run(&first_out, elapsed.count());
    } else if (elapsed.count() < experiment_length_) {
        constant_command_run(&first_out, elapsed.count());
    }
    
    if (elapsed.count() > experiment_length_){
        stop = true;
    } else { 
        stop = false;
    }
    return stop;
}