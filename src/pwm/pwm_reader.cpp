#include "pwm_reader.h"
#include <pigpio.h>
#include <cstdint>


constexpr uint64_t max_tick_value = static_cast<uint64_t>(UINT32_MAX) + 1;

PWMReader::PWMReader(unsigned int pin) : pin_(pin), rise_tick_(0), pulse_width_(0) {
    gpioSetMode(pin_, PI_INPUT);
    gpioSetPullUpDown(pin_, PI_PUD_OFF);
    gpioSetAlertFuncEx(pin_, &PWMReader::pwm_cbfunc, this);
}

PWMReader::~PWMReader() {
    gpioSetAlertFuncEx(pin_, nullptr, nullptr);
}

uint32_t PWMReader::pulse_width() const {
    return pulse_width_.load();
}

void PWMReader::pwm_cbfunc(int gpio, int level, uint32_t tick, void* userdata) {
    PWMReader* pwm_reader = static_cast<PWMReader*>(userdata);
    pwm_reader->process_edge(level, tick);
}

void PWMReader::process_edge(int level, uint32_t tick) {
    if (level == 1) {
        rise_tick_.store(tick);
    } else if (level == 0) {
        uint64_t diff = static_cast<uint64_t>(tick) - static_cast<uint64_t>(rise_tick_.load());
        uint32_t wrapped_diff = static_cast<uint32_t>(diff % max_tick_value);
        pulse_width_.store(wrapped_diff);
    }
}
