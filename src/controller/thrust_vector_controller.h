#ifndef THRUST_VECTOR_CONTROLLER_H
#define THRUST_VECTOR_CONTROLLER_H

#include "../motor_control/moteus_protocol.h"
#include "../motor_control/pi3hat_moteus_interface.h"
#include "controller.h"

using namespace mjbots;
using MoteusInterface = moteus::Pi3HatMoteusInterface;

class ThrustVectorController : public Controller
{
public:
    ThrustVectorController(std::vector<float> velocity, std::vector<float> amplitude, std::vector<float> phase,
                           float experiment_length, float startup_sequence_length = 1.0);
    void initialize(std::vector<MoteusInterface::ServoCommand> *commands);
    moteus::QueryResult get(const std::vector<MoteusInterface::ServoReply> &replies,
                            int id, int bus);
    float value_sweep(float start_value, float end_value, float elapsed_seconds, float end_time_seconds);
    void apply_constant_command(MoteusInterface::ServoCommand *command, float velocity, float amplitude, float phase);
    void startup_sequence_run(MoteusInterface::ServoCommand *command, float velocity, float elapsed_seconds);
    void multi_sequence_run(MoteusInterface::ServoCommand *command, float elapsed_seconds);
    bool run(const std::vector<MoteusInterface::ServoReply> &status,
             std::vector<MoteusInterface::ServoCommand> *output);

private:
    std::vector<float> velocity_;
    std::vector<float> amplitude_;
    std::vector<float> phase_;
    float experiment_length_;
    float startup_sequence_length_;
    int cycle_count_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
};
#endif