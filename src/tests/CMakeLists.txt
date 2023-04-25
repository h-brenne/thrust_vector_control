cmake_minimum_required(VERSION 3.0.0)
project(tests VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 11)

include_directories(../)

add_executable(thrust_vector_sequence_generator_test
    thrust_vector_sequence_generator_test.cpp
    ../controller/thrust_vector_sequence_generator.cpp
)

enable_testing()
add_test(NAME thrust_vector_sequence_generator_test COMMAND thrust_vector_sequence_generator_test)
