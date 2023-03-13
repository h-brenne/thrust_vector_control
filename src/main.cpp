#include <sys/mman.h>
#include <iostream>
#include <vector>
#include "motor_control/moteus_motor_control.h"
#include "controller/thrust_vector_controller.h"

const double PI = 3.14159265358979323846;

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

	// Test settings
	std::vector<float> velocity{35.0, 35.0, 35.0, 35.0, 35.0};
	std::vector<float> amplitude{0.0, 0.3, 0.3, 0.3, 0.0};
	std::vector<float> phase{0*PI/180, 0*PI/180, 180*PI/180, 90*PI/180, 0.0};
	float experiment_length_seconds = 2.0;
	// Lock memory for the whole process.
	LockMemory();
	ThrustVectorController controller(velocity, amplitude, phase, experiment_length_seconds);
	MoteusMotorControl motor_controller(main_cpu, can_cpu, period_s,
                    					servo_bus_map, "logs/test.csv");
	motor_controller.run(&controller);
	return 0;
}
