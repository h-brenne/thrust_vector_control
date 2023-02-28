#include <sys/mman.h>
#include <iostream>
#include <vector>
#include "motor_control/moteus_motor_control.h"
#include "controller/thrust_vector_controller.h"
#include "controller/thrust_vector_sequence_generator.h"

//const double PI = 3.14159265358979323846;

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

	float min_velocity = 40.0;
    float max_velocity = 90.0;
    float step_velocity = 5.0;
    float min_amplitude = 0.0;
    float max_amplitude = 0.26;
    float step_amplitude = 0.04;
    float min_phase = 0.0;
    float max_phase = 0.0;
    float step_phase = 1.0;

    std::vector<float> velocities;
    std::vector<float> amplitudes;
    std::vector<float> phases;
	bool enable_dual_amplitude_steps = true;
    generateThrustVectorSequence(min_velocity, max_velocity, step_velocity,
                                 min_amplitude, max_amplitude, step_amplitude,
                                 min_phase, max_phase, step_phase, enable_dual_amplitude_steps,
                                 velocities, amplitudes, phases);

	for (size_t i = 0; i < velocities.size(); ++i) {
        std::cout << "Velocity: " << velocities[i]
                  << ", Amplitude: " << amplitudes[i]
                  << ", Phase: " << phases[i] << std::endl;
    }
	float experiment_length_seconds = 180.0;

	// Lock memory for the whole process.
	//LockMemory();
	ThrustVectorController controller(velocities, amplitudes, phases, experiment_length_seconds);
	MoteusMotorControl motor_controller(main_cpu, can_cpu, period_s,
                    					servo_bus_map, "logs/test.csv");
	motor_controller.run(&controller);
	return 0;
}
