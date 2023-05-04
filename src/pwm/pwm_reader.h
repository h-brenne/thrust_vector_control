#ifndef PWM_READER_H
#define PWM_READER_H

#include <atomic>

class PWMReader {
public:
    PWMReader(unsigned int pin);
    ~PWMReader();

    uint32_t pulse_width() const;

private:
    static void pwm_cbfunc(int gpio, int level, uint32_t tick, void* userdata);
    void process_edge(int level, uint32_t tick);

    unsigned int pin_;
    std::atomic<uint32_t> rise_tick_;
    std::atomic<uint32_t> pulse_width_;
};

#endif // PWM_READER_H