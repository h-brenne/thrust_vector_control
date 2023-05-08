#!/bin/bash
sudo ./performance_governor.sh
sudo chrt -r 99 ./build/thrust_vector_controller
