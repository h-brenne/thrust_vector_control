#ifndef THRUST_VECTOR_CONTROLLER_H
#define THRUST_VECTOR_CONTROLLER_H

#include "../motor_control/moteus_protocol.h"
#include "../motor_control/pi3hat_moteus_interface.h"


using namespace mjbots;
using MoteusInterface = moteus::Pi3HatMoteusInterface;

class ThrustVectorController {
    public:
        ThrustVectorController();
        void Initialize(std::vector<MoteusInterface::ServoCommand>* commands);
        moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply>& replies,
                                int id, int bus);
        void Run(const std::vector<MoteusInterface::ServoReply>& status,
                std::vector<MoteusInterface::ServoCommand>* output);
    
    private:
        int cycle_count_;
};
#endif