#include <iostream>
#include <iterator>
#include <chrono>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <pigpio.h>
#include <signal.h>
#include "../pwm/pwm_reader.h"
#include "../motor_control/realtime.h"

bool stop = false;
std::vector<std::string> log_data;

void save_log_data(const std::string &filename) {
    std::ofstream output_file(filename);
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(std::begin(log_data), std::end(log_data), output_iterator);
}

void signal_handler(int signum) {
    stop = true;
}

int main(int argc, char *argv[]) {
    std::string log_filename;
    if (argc > 1) {
        log_filename = argv[1];
    } else {
        log_filename = "";
    }
    mjbots::moteus::ConfigureRealtime(1);

    std::vector<unsigned int> pins = {2, 3, 4, 27, 6, 13};
    if (gpioCfgClock(1,1,1) < 0) {
	    std::cerr << "pgpio clock set failed\n";		    
    }
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed\n";
        return 1;
    }

    std::vector<std::unique_ptr<PWMReader>> pwm_inputs;
    for (auto pin : pins) {
        pwm_inputs.emplace_back(std::make_unique<PWMReader>(pin));
    }

    if (log_filename != "") {
        std::ostringstream header;
        header << "Timestamp_us";
        for (size_t i = 0; i < pwm_inputs.size(); ++i) {
            header << ",Pin" << i;
        }
        log_data.push_back(header.str());
    }

    std::cout << "Starting PWM reader. Press Ctrl-C to stop.\n";

    signal(SIGINT, signal_handler);
    while (!stop) {
        std::ostringstream log_line;
        if (log_filename != "") {
            auto now = std::chrono::steady_clock::now();
            auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
            log_line << timestamp_us;
        }
        for (size_t i = 0; i < pwm_inputs.size(); ++i) {
            if (log_filename != "") {
                log_line << "," << pwm_inputs[i]->pulse_width();
            }
            else {
                std::cout << "PWM pulse width (pin:" << i << ", bcm " <<pins[i] << "): " << pwm_inputs[i]->pulse_width() << std::endl;
            }
        }
        if (log_filename != "") {
            log_data.push_back(log_line.str());
        }
        gpioDelay(2500);
    }

    gpioTerminate();

    if (log_filename != "") {
        try {
            save_log_data(log_filename);
        } catch (const std::exception &e) {
            std::cerr << "Error saving log data: " << e.what() << std::endl;
            return 1;
    }
    }
    return 0;
}
