#include "pwm_reader.h"
#include <pigpio.h>

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
        pulse_width_.store(tick - rise_tick_.load());
    }
}
