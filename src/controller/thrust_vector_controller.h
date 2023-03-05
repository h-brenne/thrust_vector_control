#ifndef THRUST_VECTOR_CONTROLLER_H
#define THRUST_VECTOR_CONTROLLER_H

#include "../motor_control/moteus_protocol.h"
#include "../motor_control/pi3hat_moteus_interface.h"
#include "controller.h"

using namespace mjbots;
using MoteusInterface = moteus::Pi3HatMoteusInterface;

class ThrustVectorController : public Controller {
    public:
        ThrustVectorController();
        void initialize(std::vector<MoteusInterface::ServoCommand>* commands);
        moteus::QueryResult get(const std::vector<MoteusInterface::ServoReply>& replies,
                                int id, int bus);
        bool run(const std::vector<MoteusInterface::ServoReply>& status,
                std::vector<MoteusInterface::ServoCommand>* output);
    private:
        int cycle_count_;
        bool stop_;
        std::chrono::time_point<std::chrono::steady_clock> start_time_;
};
#endif