// thrust_vector_sequence_generator.h
#pragma once

#include <vector>

const float PI = 3.14159265359;

void generateThrustVectorSequence(float min_velocity, float max_velocity, float step_velocity,
                                  float min_amplitude, float max_amplitude, float step_amplitude,
                                  float min_phase, float max_phase, float step_phase, bool enable_dual_amplitude_steps,
                                  std::vector<float>& velocities,
                                  std::vector<float>& amplitudes,
                                  std::vector<float>& phases);