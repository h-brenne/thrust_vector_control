cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_SYSTEM_PROCESSOR ARM) 
project(thrust_vector_control VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "-mcpu=cortex-a53 -Wno-psabi" CACHE STRING "compile flags" FORCE)
include(CTest)
enable_testing()

add_library(date INTERFACE)
target_include_directories(date INTERFACE third_party_libraries/)

include_directories(src /opt/vc/include)
link_directories(/opt/vc/lib)

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(thrust_vector_control PUBLIC "-pthread")
endif()

add_executable(thrust_vector_control 
    src/motor_control/moteus_motor_control.cpp
    src/pi3hat/pi3hat.cpp
    src/controller/thrust_vector_controller.cpp
    src/main.cpp)

if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(thrust_vector_control "${CMAKE_THREAD_LIBS_INIT}")
  endif()
target_link_libraries(thrust_vector_control -lbcm_host date)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
