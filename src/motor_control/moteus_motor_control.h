// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MOTEUS_MOTOR_CONTROL_H
#define MOTEUS_MOTOR_CONTROL_H

#include <sys/mman.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <iterator>

#include "moteus_protocol.h"
#include "pi3hat_moteus_interface.h"
#include "../controller/controller.h"
using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

class MoteusMotorControl {
	private:
		const int main_cpu_;
		const int can_cpu_;
		const float period_s_;
		const std::vector<std::pair<int, int>> servo_bus_map_;
		MoteusInterface moteus_interface_;
		std::string log_file_;
		
		MoteusInterface::Options get_initialization_options(int can_cpu);
	public:
		MoteusMotorControl(const int main_cpu, const int can_cpu,
                    const float period_s,
                    const std::vector<std::pair<int, int>> servo_bus_map, std::string log_file = "");
		void stop();
		// Has to be implemented here for linker to work with template function
		// Open to other solutions
		//template <typename Controller>
		void run(Controller *controller);
};

#endif