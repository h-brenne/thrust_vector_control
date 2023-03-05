#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../motor_control/moteus_protocol.h"
using namespace mjbots;
using MoteusInterface = moteus::Pi3HatMoteusInterface;

class Controller {
public:
    virtual bool run(const std::vector<MoteusInterface::ServoReply>& status,
                std::vector<MoteusInterface::ServoCommand>* output) = 0;
    virtual void initialize(std::vector<MoteusInterface::ServoCommand>* commands) = 0;
};

#endif