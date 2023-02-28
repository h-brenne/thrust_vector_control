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


std::pair<double, double> MinMaxVoltage(
		const std::vector<MoteusInterface::ServoReply> &r)
{
	double rmin = std::numeric_limits<double>::infinity();
	double rmax = -std::numeric_limits<double>::infinity();

	for (const auto &i : r)
	{
		if (i.result.voltage > rmax)
		{
			rmax = i.result.voltage;
		}
		if (i.result.voltage < rmin)
		{
			rmin = i.result.voltage;
		}
	}

	return std::make_pair(rmin, rmax);
}

MoteusMotorControl::MoteusMotorControl(const int main_cpu, const int can_cpu, const float period_s, const std::vector<std::pair<int, int>> servo_bus_map)
	: main_cpu_(main_cpu)
	, can_cpu_(can_cpu)
	, period_s_(period_s)
	, servo_bus_map_(servo_bus_map)
	, moteus_interface_{get_initialization_options(can_cpu)}
	{
	moteus::ConfigureRealtime(main_cpu);
}

template <typename Controller>
void MoteusMotorControl::Run(Controller *controller)
{
	std::vector<MoteusInterface::ServoCommand> commands;
	for (const auto &pair : controller->servo_bus_map())
	{
		commands.push_back({});
		commands.back().id = pair.first;
		commands.back().bus = pair.second;
	}

	std::vector<MoteusInterface::ServoReply> replies{commands.size()};
	std::vector<MoteusInterface::ServoReply> saved_replies;

	controller->Initialize(&commands);

	MoteusInterface::Data moteus_data;
	moteus_data.commands = {commands.data(), commands.size()};
	moteus_data.replies = {replies.data(), replies.size()};

	std::future<MoteusInterface::Output> can_result;

	const auto period =
			std::chrono::microseconds(static_cast<int64_t>(period_s_ * 1e6));
	auto next_cycle = std::chrono::steady_clock::now() + period;

	const auto status_period = std::chrono::milliseconds(100);
	auto next_status = next_cycle + status_period;
	uint64_t cycle_count = 0;
	double total_margin = 0.0;
	uint64_t margin_cycles = 0;

	// We will run at a fixed cycle time.
	while (true)
	{
		cycle_count++;
		margin_cycles++;
		{
			const auto now = std::chrono::steady_clock::now();
			if (now > next_status)
			{
				// NOTE: iomanip is not a recommended pattern.  We use it here
				// simply to not require any external dependencies, like 'fmt'.
				const auto volts = MinMaxVoltage(saved_replies);
				const std::string modes = [&]()
				{
					std::ostringstream result;
					result.precision(4);
					result << std::fixed;
					for (const auto &item : saved_replies)
					{
						result << item.id << "/"
									<< item.bus << "/"
									<< static_cast<int>(item.result.mode) << "/"
									<< item.result.position << " ";
					}
					return result.str();
				}();
				std::cout << std::setprecision(6) << std::fixed
									<< "Cycles " << cycle_count
									<< "  margin: " << (total_margin / margin_cycles)
									<< std::setprecision(1)
									<< "  volts: " << volts.first << "/" << volts.second
									<< "  modes: " << modes
									<< "   \r";
				std::cout.flush();
				next_status += status_period;
				total_margin = 0;
				margin_cycles = 0;
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

		controller->Run(saved_replies, &commands);

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
}

MoteusInterface::Options MoteusMotorControl::get_initialization_options(int can_cpu)
{
	MoteusInterface::Options moteus_options;
	moteus_options.cpu = can_cpu;
	return moteus_options;
}
