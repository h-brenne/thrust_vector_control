#include <stdexcept>

#include "thrust_vector_controller.h"
//using namespace mjbots;
//using MoteusInterface = moteus::Pi3HatMoteusInterface;

ThrustVectorController::ThrustVectorController(){};

void ThrustVectorController::Initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    stop_ = false;
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt16;
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
}

moteus::QueryResult ThrustVectorController::Get(const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) {
    for (const auto& item : replies) {
        if (item.id == id && item.bus == bus) { return item.result; }
    }
    return {};
}

void ThrustVectorController::Run(const std::vector<MoteusInterface::ServoReply>& status,
        std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;

    // This is where your control loop would go.

    if (stop_ || cycle_count_ < 5 || cycle_count_ > 2000) {
        for (auto& cmd : *output) {
            // We start everything with a stopped command to clear faults.
            cmd.mode = moteus::Mode::kStopped;
        }
    } else {
    auto& first_out = output->at(0);  // We constructed this, so we know the order.
    first_out.mode = moteus::Mode::kPosition;
    first_out.position.position = std::numeric_limits<double>::quiet_NaN();
    first_out.position.velocity = 1;

    }
}

void ThrustVectorController::Stop() {
    stop_ = true;
}
