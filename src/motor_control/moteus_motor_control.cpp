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
#include <ctime>
#include "date.h"
#include <signal.h>
//#include <fmt/core.h> 
using namespace date;

MoteusMotorControl::MoteusMotorControl(const int main_cpu, const int can_cpu, const float period_s,
									   const std::vector<std::pair<int, int>> servo_bus_map, std::string log_file)
	: main_cpu_(main_cpu)
	, can_cpu_(can_cpu)
	, period_s_(period_s)
	, servo_bus_map_(servo_bus_map)
	, moteus_interface_{get_initialization_options(can_cpu)}
	, log_file_(log_file)
	{
		moteus::ConfigureRealtime(main_cpu);
		
}

bool MoteusMotorControl::stop_ = false;

void MoteusMotorControl::stop(int signum) {
	std::cout << "Stopping" << std::endl;
	stop_ = true;
}
MoteusInterface::Options MoteusMotorControl::get_initialization_options(int can_cpu)
{
	MoteusInterface::Options moteus_options;
	moteus_options.cpu = can_cpu;
	return moteus_options;
}

void MoteusMotorControl::run(Controller *controller) {
	std::vector<MoteusInterface::ServoCommand> commands;
	for (const auto &pair : servo_bus_map_)
	{
		commands.push_back({});
		commands.back().id = pair.first;
		commands.back().bus = pair.second;
	}

	std::vector<MoteusInterface::ServoReply> replies{commands.size()};
	std::vector<MoteusInterface::ServoReply> saved_replies;

	controller->initialize(&commands);
	MoteusInterface::Data moteus_data;
	moteus_data.commands = {commands.data(), commands.size()};
	moteus_data.replies = {replies.data(), replies.size()};

	std::future<MoteusInterface::Output> can_result;

	const auto period =
			std::chrono::microseconds(static_cast<int64_t>(period_s_ * 1e6));
	auto next_cycle = std::chrono::steady_clock::now() + period;
	
	uint64_t cycle_count = 0;
	double total_margin = 0.0;

	std::vector<std::string> log_data{
		"Time,ID,Bus,Mode,Velocity,Torque,ControlVelocity,VelocityCommand,AmplitudeCommand,PhaseCommand,Temperature,Voltage"};

	int stop_next = false;

	signal(SIGINT, stop);
	while (!stop_next)
	{
		cycle_count++;
		{
			const auto now = std::chrono::steady_clock::now();
			auto now_system_clock = std::chrono::system_clock::now();
			std::time_t now_t = std::chrono::system_clock::to_time_t(now_system_clock);
			// Push data to vector if logging is enabled
			if (!log_file_.empty()) {
				for (const auto &item : saved_replies)
				{
					MoteusInterface::ServoCommand* current_command = nullptr;
					for (auto &command : commands)
					{
						if (command.id == item.id && command.bus == item.bus)
						{
							current_command = &command;
							break;
						}
					}
					
					if (current_command)
					{
						float velocity_cmd = current_command->position.velocity;
						float amplitude_cmd = current_command->position.sinusoidal_amplitude;
						float phase_cmd = current_command->position.sinusoidal_phase;

						std::ostringstream result;
						result.precision(5);
						result << std::fixed;

						result << now_system_clock << ","
							<< item.id << ","
							<< item.bus << ","
							<< static_cast<int>(item.result.mode) << ","
							<< item.result.velocity << ","
							<< item.result.torque << ","
							<< item.result.control_velocity << ","
							<< velocity_cmd << ","
							<< amplitude_cmd << ","
							<< phase_cmd << ","
							<< item.result.temperature << ","
							<< item.result.voltage;

						log_data.push_back(result.str());
					}
				}
			}

			int skip_count = 0;
			while (now > next_cycle)
			{
				skip_count++;
				next_cycle += period;
			}
			if (skip_count)
			{
				std::cout << "\nSkipped " << skip_count << " cycles\n";
			}
		}
		// Wait for the next control cycle to come up.
		{
			const auto pre_sleep = std::chrono::steady_clock::now();
			std::this_thread::sleep_until(next_cycle);
			const auto post_sleep = std::chrono::steady_clock::now();
			std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
			total_margin += elapsed.count();
		}
		next_cycle += period;

		bool controller_stop = false;
		if (cycle_count < 5) {
			for (auto& cmd : commands) {
				// We start everything with a stopped command to clear faults.
				cmd.mode = moteus::Mode::kStopped;
			}
		} else {
			// Run the controller, which decides when to stop the loop
			
			controller_stop = controller->run(saved_replies, &commands);
		}
		
		if (MoteusMotorControl::stop_ or controller_stop) {
			for (auto& cmd : commands) {
				cmd.mode = moteus::Mode::kStopped;
			}
			stop_next = true;
		}
		

		if (can_result.valid())
		{
			// Now we get the result of our last query and send off our new
			// one.
			const auto current_values = can_result.get();

			// We copy out the results we just got out.
			const auto rx_count = current_values.query_result_size;
			saved_replies.resize(rx_count);
			std::copy(replies.begin(), replies.begin() + rx_count,
								saved_replies.begin());
		}

		// Then we can immediately ask them to be used again.
		auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
		moteus_interface_.Cycle(
				moteus_data,
				[promise](const MoteusInterface::Output &output)
				{
					// This is called from an arbitrary thread, so we just set
					// the promise value here.
					promise->set_value(output);
				});
		can_result = promise->get_future();
	}

	//Save log file on exit
	if (!log_file_.empty()) {
		std::ofstream output_file(log_file_);
		std::ostream_iterator<std::string> output_iterator(output_file, "\n");
		std::copy(std::begin(log_data), std::end(log_data), output_iterator);
	}
}