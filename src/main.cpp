#include <sys/mman.h>
#include <iostream>
#include "motor_control/moteus_motor_control.h"
#include "controller/thrust_vector_controller.h"

void LockMemory()
	{
		// We lock all memory so that we don't end up having to page in
		// something later which can take time.
		{
			const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
			if (r < 0)
			{
				throw std::runtime_error("Error locking memory");
			}
		}
	}

int main(int argc, char **argv) {

	int main_cpu = 1;
	int can_cpu = 2;
	float period_s = 0.0004;
	std::vector<std::pair<int, int>> servo_bus_map = {{1,1}};

	// Lock memory for the whole process.
	LockMemory();
	ThrustVectorController controller;
	MoteusMotorControl motor_controller(main_cpu, can_cpu, period_s,
                    					servo_bus_map, "logs/test.csv");
	motor_controller.run(&controller);
	return 0;
}
