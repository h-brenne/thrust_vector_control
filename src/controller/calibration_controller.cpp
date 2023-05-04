#include <stdexcept>
#include <iostream>
#include <vector>
#include "calibration_controller.h"


CalibrationController::CalibrationController(std::vector<float> velocity, std::vector<float> amplitude, std::vector<float> phase, float experiment_length, float startup_sequence_length)
    : velocity_(velocity), amplitude_(amplitude), phase_(phase), experiment_length_(experiment_length), startup_sequence_length_(startup_sequence_length)
{
    if (!(velocity_.size() == amplitude_.size() && amplitude_.size() == phase_.size()))
    {
        throw std::invalid_argument("Velocit, amplitude and phase vectors must be same length");
    }
};

void CalibrationController::initialize(std::vector<MoteusInterface::ServoCommand> *commands)
{
    cycle_count_ = 0;
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt8;
    res.velocity = moteus::Resolution::kFloat;
    res.feedforward_torque = moteus::Resolution::kIgnore;
    res.sinusoidal_amplitude = moteus::Resolution::kInt16;
    res.sinusoidal_phase = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kIgnore;
    res.kd_scale = moteus::Resolution::kIgnore;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;

    moteus::QueryCommand query_cmd;
    query_cmd.mode = moteus::Resolution::kInt16;
    query_cmd.position = moteus::Resolution::kIgnore;
    query_cmd.velocity = moteus::Resolution::kFloat;
    query_cmd.torque = moteus::Resolution::kInt16;
    query_cmd.q_current = moteus::Resolution::kIgnore;
    query_cmd.d_current = moteus::Resolution::kIgnore;
    query_cmd.rezero_state = moteus::Resolution::kInt8;
    query_cmd.voltage = moteus::Resolution::kInt8;
    query_cmd.temperature = moteus::Resolution::kInt8;
    query_cmd.fault = moteus::Resolution::kInt8;
    query_cmd.control_velocity = moteus::Resolution::kFloat;

    for (auto &cmd : *commands)
    {
        cmd.resolution = res;
        cmd.query = query_cmd;
    }
    start_time_ = std::chrono::steady_clock::now();
}

moteus::QueryResult CalibrationController::get(const std::vector<MoteusInterface::ServoReply> &replies, int id, int bus)
{
    for (const auto &item : replies)
    {
        if (item.id == id && item.bus == bus)
        {
            return item.result;
        }
    }
    return {};
}

float CalibrationController::value_sweep(float start_value, float end_value, float elapsed_seconds, float end_time_seconds)
{
    return start_value + (elapsed_seconds / end_time_seconds) * (end_value - start_value);
}

void CalibrationController::apply_constant_command(MoteusInterface::ServoCommand *command, float velocity, float amplitude, float phase)
{
    command->mode = moteus::Mode::kSinusoidal;
    command->position.position = std::numeric_limits<double>::quiet_NaN();
    command->position.maximum_torque = std::numeric_limits<double>::quiet_NaN();
    command->position.velocity = velocity;
    command->position.sinusoidal_amplitude = amplitude;
    command->position.sinusoidal_phase = phase;
    return;
}

void CalibrationController::startup_sequence_run(MoteusInterface::ServoCommand *command, float velocity, float elapsed_seconds)
{
    command->mode = moteus::Mode::kSinusoidal;
    command->position.position = std::numeric_limits<double>::quiet_NaN();
    float end_velocity = velocity;
    command->position.maximum_torque = 0.5;
    command->position.sinusoidal_amplitude = 0.0;
    command->position.sinusoidal_phase = 0.0;
    command->position.velocity = value_sweep(0, end_velocity, elapsed_seconds, startup_sequence_length_);
    return;
}

void CalibrationController::multi_sequence_run(MoteusInterface::ServoCommand *command, float elapsed_seconds)
{
    float elapsed_fraction = elapsed_seconds / experiment_length_;
    int command_index = int(elapsed_fraction * velocity_.size());

    // Do command
    apply_constant_command(command, velocity_[command_index], amplitude_[command_index], phase_[command_index]);
    return;
}

bool CalibrationController::run(const std::vector<MoteusInterface::ServoReply> &status,
                                 std::vector<MoteusInterface::ServoCommand> *output)
{
    bool stop = false;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;

    // Calibration only controls first motor
    auto &first_out = output->at(0); 

    if (elapsed.count() < startup_sequence_length_)
    {
        // Startup sequence
        // Makes sure that the hinged rotor folds out more gracefully
        startup_sequence_run(&first_out, velocity_[0], elapsed.count());
    }
    else if (elapsed.count() - startup_sequence_length_ < experiment_length_)
    {
        multi_sequence_run(&first_out, elapsed.count() - startup_sequence_length_);
    }

    if (elapsed.count() - startup_sequence_length_ > experiment_length_)
    {
        stop = true;
    }
    else
    {
        stop = false;
    }
    return stop;
}