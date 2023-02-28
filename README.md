# Thrust Vector Control

This repository contains a work-in-progress program for controlling the Moteus open-source motor driver to enable thrust vector control for aerial vehicles with just a single actuator. The program is designed to run on a Raspberry Pi with the Moteus Pi3Hat CAN adapter.
## Table of Contents 
- [Dependencies](https://chat.openai.com/chat/eed821db-2d68-44d2-8c55-450cdada0cf2#dependencies) 
- [Building the Code](https://chat.openai.com/chat/eed821db-2d68-44d2-8c55-450cdada0cf2#building-the-code) 
- [Usage](https://chat.openai.com/chat/eed821db-2d68-44d2-8c55-450cdada0cf2#usage)
## Dependencies

Please refer to the `CMakeLists.txt` file in the repository for dependencies. You will need to have all dependencies installed on your system before building and running the program.
## Building the Code

To build the code, follow these steps:
1. Clone the repository:

```bash

git clone https://github.com/h-brenne/thrust_vector_control.git
```


1. Navigate to the repository directory:

```bash

cd thrust_vector_control
```


1. Create a build directory and navigate to it:

```bash

mkdir build && cd build
```


1. Run CMake to generate the build files:

```

cmake ..
```


1. Build the program:

```go

make
```


## Usage
- After building the code, you can run the program by executing the generated binary file. Please ensure that your Moteus motor driver, Raspberry Pi, and aerial vehicle setup are properly connected and configured before running the program. The Moteus controller should be connected to the "JC1" CAN port on the Pi3Hat.

This program is designed for two primary purposes:
1. Implementing thrust vector control for aerial vehicles with just a single actuator.
2. Collecting calibration data by generating a sequence of thrust vector parameters.

To run the program, execute the following command from the `build` directory:

```bash

./thrust_vector_control
```



The program will output the velocity, amplitude, and phase of the generated thrust vector sequence. It will also log data to a specified log file. This data can be used for calibration and analysis of the aerial vehicle's performance.

Please note that this repository is a work in progress and does not have the ambition to be highly reusable. It is tailored for specific use cases and may require modification for other purposes. The target audience is people who are already familiar with thrust vector control, aerial vehicles, and motor control concepts.
