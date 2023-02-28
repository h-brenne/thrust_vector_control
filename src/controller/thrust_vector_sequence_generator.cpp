// thrust_vector_sequence_generator.cpp
#include "thrust_vector_sequence_generator.h"

void generateThrustVectorSequence(float min_velocity, float max_velocity, float step_velocity,
                                  float min_amplitude, float max_amplitude, float step_amplitude,
                                  float min_phase, float max_phase, float step_phase, bool enable_dual_amplitude_steps,
                                  std::vector<float>& velocities,
                                  std::vector<float>& amplitudes,
                                  std::vector<float>& phases) {
    for (float velocity = min_velocity; velocity <= max_velocity; velocity += step_velocity) {
        for (float amplitude = min_amplitude; amplitude <= max_amplitude; amplitude += step_amplitude) {
            for (float phase = min_phase; phase <= max_phase; phase += step_phase) {
                velocities.push_back(velocity);
                amplitudes.push_back(amplitude);
                phases.push_back(phase);
            }
        }
        
        // Move amplitude back to minimum in steps. This is to reduce stresses during calibration
        if(enable_dual_amplitude_steps) {
            for (float amplitude = max_amplitude; amplitude >= min_amplitude; amplitude -= step_amplitude) {
                for (float phase = min_phase; phase <= max_phase; phase += step_phase) {
                    velocities.push_back(velocity);
                    amplitudes.push_back(amplitude);
                    phases.push_back(phase);
                }
            }
        }
    }
}