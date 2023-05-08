#include <sys/mman.h>
#include <iostream>
#include <vector>
#include "motor_control/moteus_motor_control.h"
#include "controller/pwm_input_controller.h"
#include "controller/thrust_vector_sequence_generator.h"

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
	float period_s = 0.0008;//0.0004 is 2000hz;
	std::vector<std::pair<int, int>> servo_bus_map = {{1,3},{2,3}};

	
	int pin_motor1_thrust = 2;
	int pin_motor2_thrust = 3;
	int pin_motor1_elevation = 4;
	int pin_motor2_elevation = 27;
	int pin_motor1_azimuth = 6;
	int pin_motor2_azimuth = 13;

	PWMInputController controller(pin_motor1_thrust, pin_motor2_thrust, pin_motor1_elevation, 
								  pin_motor2_elevation, pin_motor1_azimuth, pin_motor2_azimuth);
	MoteusMotorControl motor_controller(main_cpu, can_cpu, period_s,
                    					servo_bus_map, "logs/test.csv");
	// Lock memory for the whole process.
	LockMemory();
	motor_controller.run(&controller);
	return 0;
}

