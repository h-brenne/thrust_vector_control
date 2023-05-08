#ifndef PWM_INPUT_CONTROLLER_H
#define PWM_INPUT_CONTROLLER_H

#include "../motor_control/moteus_protocol.h"
#include "../motor_control/pi3hat_moteus_interface.h"
#include "controller.h"
#include "../pwm/pwm_reader.h"

using namespace mjbots;
using MoteusInterface = moteus::Pi3HatMoteusInterface;

class PWMInputController : public Controller
{
public:
    PWMInputController(unsigned int pin_motor1_thrust, unsigned int pin_motor2_thrust,
                       unsigned int pin_motor1_elevation, unsigned int pin_motor2_elevation,
                       unsigned int pin_motor1_azimuth, unsigned int pin_motor2_azimuth,
                       const std::string& log_filename = "");
    ~PWMInputController();
    void initialize(std::vector<MoteusInterface::ServoCommand> *command);
    void save_log_data(const std::string &filename);
    float map(float x, float in_min, float in_max, float out_min, float out_max);
    void apply_motor_command(MoteusInterface::ServoCommand *command, float velocity, float amplitude, float phase);
    bool run(const std::vector<MoteusInterface::ServoReply> &status,
             std::vector<MoteusInterface::ServoCommand> *output);

private:
    std::array<std::unique_ptr<PWMReader>, 6> pwm_readers_;
    std::string log_filename_;
    std::vector<std::string> log_data_;
};

#endif
