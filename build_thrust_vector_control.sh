#!/bin/bash

g++ -O2 -g -Wall --std=c++11 \
    -mcpu=cortex-a53 \
    -Wno-psabi \
    -I src -I /opt/vc/include -L /opt/vc/lib \
    -o thrust_vector_control \
    src/main.cpp \
    src/motor_control/moteus_motor_control.cpp \
    src/pi3hat/pi3hat.cpp \
    src/thrust_vector_controller/thrust_vector_controller.cpp \
    -lbcm_host \
    -lpthread
