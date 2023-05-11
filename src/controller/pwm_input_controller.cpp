#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <chrono>
#include <pigpio.h>
#include "pwm_input_controller.h"
#include "../motor_control/realtime.h"

float clamp(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

PWMInputController::PWMInputController(unsigned int pin_motor1_thrust, unsigned int pin_motor2_thrust,
                                       unsigned int pin_motor1_elevation, unsigned int pin_motor2_elevation,
                                       unsigned int pin_motor1_azimuth, unsigned int pin_motor2_azimuth,
                                       const std::string& log_filename)
: log_filename_(log_filename)
{
    // Run on core 3, pi3hat extra stuff is not used
    mjbots::moteus::ConfigureRealtime(3);
    // Set gpio sampling rate(first parameter in us). Seconds parameter is PWM or PCM,
    // didn't see any difference in performance
    if (gpioCfgClock(1,1,1) < 0) {
	    std::cerr << "pgpio clock set failed\n";		    
    }
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed\n";
    }
    pwm_readers_ = {{
        std::make_unique<PWMReader>(pin_motor1_thrust),
        std::make_unique<PWMReader>(pin_motor2_thrust),
        std::make_unique<PWMReader>(pin_motor1_elevation),
        std::make_unique<PWMReader>(pin_motor2_elevation),
        std::make_unique<PWMReader>(pin_motor1_azimuth),
        std::make_unique<PWMReader>(pin_motor2_azimuth)
    }};

    if (!log_filename_.empty()) {
        std::ostringstream header;
        header << "Timestamp_us";
        for (size_t i = 0; i < pwm_readers_.size(); ++i) {
            header << ",Pin" << i;
        }
        log_data_.push_back(header.str());
    }
}

void PWMInputController::initialize(std::vector<MoteusInterface::ServoCommand> *commands) {
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
}
PWMInputController::~PWMInputController() {
    // There are still ISR callbacks active, as the PWMReaders haven't gone out of scope yet
    // Might move this to main
    gpioTerminate();

    if (!log_filename_.empty()) {
        try {
            save_log_data(log_filename_);
        } catch (const std::exception &e) {
            std::cerr << "Error saving log data in destructor: " << e.what() << std::endl;
            // Do not rethrow exception from the destructor
        }
    }
}

void PWMInputController::save_log_data(const std::string &filename) {
    std::ofstream output_file(filename);
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(std::begin(log_data_), std::end(log_data_), output_iterator);
}

float PWMInputController::map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PWMInputController::apply_motor_command(MoteusInterface::ServoCommand *command, float velocity, float amplitude, float phase)
{
    command->mode = moteus::Mode::kSinusoidal;
    command->position.position = std::numeric_limits<double>::quiet_NaN();
    command->position.maximum_torque = std::numeric_limits<double>::quiet_NaN();
    command->position.velocity = velocity;
    command->position.sinusoidal_amplitude = amplitude;
    command->position.sinusoidal_phase = phase;
    return;
}


bool PWMInputController::run(const std::vector<MoteusInterface::ServoReply> &status,
             std::vector<MoteusInterface::ServoCommand> *output) {
    //auto now = std::chrono::steady_clock::now();
    //std::chrono::duration<double> elapsed = now - start_time_;

    // Read PWM inputs
    std::vector<uint32_t> pulse_widths;
    for (const auto &pwm_reader : pwm_readers_) {
        pulse_widths.push_back(pwm_reader->pulse_width());
    }

    // Log pwm if enabled
    if (!log_filename_.empty()) {
        std::ostringstream log_line;
        auto now = std::chrono::steady_clock::now();
        auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        log_line << timestamp_us;
        for (size_t i = 0; i < pulse_widths.size(); ++i) {
            log_line << "," << pulse_widths[i];
        }
        log_data_.push_back(log_line.str());
    }
    // Check if disarmed
    if (pulse_widths[0] < 970 or pulse_widths[1] < 970) {
        if (output->size() == 1) {
            output->at(0).mode = moteus::Mode::kStopped;
        } else if (output->size() == 2) {
            output->at(0).mode = moteus::Mode::kStopped;
            output->at(1).mode = moteus::Mode::kStopped;
        }
        return false;
    }

    // Map pulse widths to thrust, elevation, and azimuth
    float min_thrust = 0.1;
    float max_thrust = 8.0;
    float max_elevation = 15*M_PI/180;
    float thrust1 = map(pulse_widths[0], 1000, 2000, min_thrust, max_thrust);
    float thrust2 = map(pulse_widths[1], 1000, 2000, min_thrust, max_thrust);
    float elevation1 = map(pulse_widths[2], 1000, 2000, 0, max_elevation);
    float elevation2 = map(pulse_widths[3], 1000, 2000, 0, max_elevation);
    float azimuth1 = map(pulse_widths[4], 1000, 2000, -M_PI, M_PI);
    float azimuth2 = map(pulse_widths[5], 1000, 2000, -M_PI, M_PI);

    // Assure that all values are within there bounds in case of bad/ unexpected pwm values.
    thrust1 = clamp(thrust1, min_thrust, max_thrust);
    thrust2 = clamp(thrust2, min_thrust, max_thrust);
    elevation1 = clamp(elevation1, 0.0, max_elevation);
    elevation2 = clamp(elevation2, 0.0, max_elevation);
    azimuth1 = clamp(azimuth1, -M_PI, M_PI);
    azimuth2 = clamp(azimuth2, -M_PI, M_PI);

    float a = 1.027, b = -1.198; // Coefficients for force = a^velocity + b
    // Assure that velocity is always positive, motor driver handles direction
    float velocity1 = std::max(log((thrust1 - b)) / log(a), 0.0);
    float velocity2 = std::max(log((thrust2 - b)) / log(a), 0.0);


    float amplitude_a = 77.526; // Coefficient for elevation = a * amplitude
    float amplitude1 = (elevation1 * 180 / M_PI) / amplitude_a;
    float amplitude2 = (elevation2 * 180 / M_PI) / amplitude_a;

    float phase_offset = -M_PI/2; // Constant offset for phase = azimuth + constant
    // The same phase offset is used for both rotors, as their rotation frames are opposite already
    float phase1 = azimuth1 + phase_offset;
    float phase2 = azimuth2 + phase_offset;

    // Assure that command mapping is between expected bounds
    float min_velocity = std::max(log((min_thrust - b)) / log(a), 0.0);; // Minimum velocity when not disarmed
    float max_velocity = 90.0;
    float max_amplitude = 0.2;
    velocity1 = clamp(velocity1, min_velocity, max_velocity);
    velocity2 = clamp(velocity2, min_velocity, max_velocity);
    amplitude1 = clamp(amplitude1, 0.0, max_amplitude);
    amplitude2 = clamp(amplitude2, 0.0, max_amplitude);
    //A high or wrong phase should not be dangerous

    /*std::cout << "thrust1: " << thrust1 << std::endl;
    std::cout << "thrust2: " << thrust2 << std::endl;
    std::cout << "elevation1: " << elevation1*180/M_PI << std::endl;
    std::cout << "elevation2: " << elevation2*180/M_PI << std::endl;
    std::cout << "azimuth1: " << azimuth1*180/M_PI << std::endl;
    std::cout << "azimuth2: " << azimuth2*180/M_PI << std::endl;

    std::cout << "velocity1: " << velocity1 << std::endl;
    std::cout << "velocity2: " << velocity2 << std::endl;
    std::cout << "amplitude1: " << amplitude1 << std::endl;
    std::cout << "amplitude2: " << amplitude2 << std::endl;
    std::cout << "phase1: " << phase1 << std::endl;
    std::cout << "phase2: " << phase2 << std::endl;*/

    if (output->size() == 1) {
        apply_motor_command(&output->at(0), velocity1, amplitude1, phase1);
    } else if (output->size() == 2) {
        apply_motor_command(&output->at(0), velocity1, amplitude1, phase1);
        apply_motor_command(&output->at(1), velocity2, amplitude2, phase2);
    } else {
        std::cout << "Invalid number of motors" << std::endl;
    }
    return false;
}
