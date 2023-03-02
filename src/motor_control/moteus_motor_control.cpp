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
#include "moteus_motor_control.h"

MoteusMotorControl::MoteusMotorControl(const int main_cpu, const int can_cpu, const float period_s, const std::vector<std::pair<int, int>> servo_bus_map)
	: main_cpu_(main_cpu)
	, can_cpu_(can_cpu)
	, period_s_(period_s)
	, servo_bus_map_(servo_bus_map)
	, moteus_interface_{get_initialization_options(can_cpu)}
	{
		moteus::ConfigureRealtime(main_cpu);
}

MoteusInterface::Options MoteusMotorControl::get_initialization_options(int can_cpu)
{
	MoteusInterface::Options moteus_options;
	moteus_options.cpu = can_cpu;
	return moteus_options;
}