// thrust_vector_sequence_generator_test.cpp
#include "../src/controller/thrust_vector_sequence_generator.h"
#include <iostream>
#include <cassert>
void test_generateThrustVectorSequence_with_custom_parameters() {
    
}

void test_generateThrustVectorSequence() {
    float min_velocity = 20.0;
    float max_velocity = 80.0;
    float step_velocity = 5.0;
    float min_amplitude = 0.0;
    float max_amplitude = 0.4;
    float step_amplitude = 0.1;
    float min_phase = 0.0;
    float max_phase = 360.0 * PI / 180;
    float step_phase = 90.0 * PI / 180;

    std::vector<float> velocities;
    std::vector<float> amplitudes;
    std::vector<float> phases;

    generateThrustVectorSequence(min_velocity, max_velocity, step_velocity,
                                 min_amplitude, max_amplitude, step_amplitude,
                                 min_phase, max_phase, step_phase, false,
                                 velocities, amplitudes, phases);

    size_t expected_size = ((max_velocity - min_velocity) / step_velocity + 1) *
                           ((max_amplitude - min_amplitude) / step_amplitude + 1) *
                           ((max_phase - min_phase) / step_phase + 1);

    assert(velocities.size() == expected_size);
    assert(amplitudes.size() == expected_size);
    assert(phases.size() == expected_size);

    size_t index = 0;
    for (float velocity = min_velocity; velocity <= max_velocity; velocity += step_velocity) {
        for (float amplitude = min_amplitude; amplitude <= max_amplitude; amplitude += step_amplitude) {
            for (float phase = min_phase; phase <= max_phase; phase += step_phase) {
                assert(velocities[index] == velocity);
                assert(amplitudes[index] == amplitude);
                assert(phases[index] == phase);
                index++;
            }
        }
    }
}

int main() {
    test_generateThrustVectorSequence();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}