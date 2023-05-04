# src/hardware_tests/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.0)
project(hardware_tests VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 14)

include_directories(../)

add_executable(pwm_test
    main_pwm_test.cpp
)

target_link_libraries(pwm_test pigpio -lbcm_host date pwm)