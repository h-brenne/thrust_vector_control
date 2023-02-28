cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_SYSTEM_PROCESSOR ARM) 
project(thrust_vector_control VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 11)

include(CTest)
enable_testing()

include_directories(src)

add_executable(thrust_vector_control 
    src/motor_control/moteus_motor_control.cpp
    src/pi3hat/pi3hat.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
