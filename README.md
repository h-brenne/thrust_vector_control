# Thrust Vector Control
<img src="https://user-images.githubusercontent.com/12870693/234275380-4d819c23-d1b8-4313-8791-9a2e914ad55a.png" width="40%" height="40%"><img src="https://user-images.githubusercontent.com/12870693/234279627-db8c392b-cda7-4913-9b33-a430926070d3.png" width="40%" height="40%">

This repository contains a work-in-progress program for controlling the Moteus open-source motor driver to enable thrust vector control for 
aerial vehicles with just a single actuator. The program is designed to run on a Raspberry Pi with the Moteus Pi3Hat CAN adapter and custom moteus firmware [h-brenne/moteus-vector-uav](https://github.com/h-brenne/moteus_vector_uav).

The repo consists of a cpp project, and python scripts to analyze data.

There are two main operation modes, **calibration** and the **thrust vector controller**. In **calibration**, a sequence of motor commands are sent. 
Using data from a 6-axis force/torque sensor, the relationship between motor commands and thrust vector can be made.
The **thrust vector controller** takes thrust vector setpoints, and sends motor commands based on the relationships found from calibration. These
thrust vector setpoints will typcially come from a flight controller.
## Table of Contents 
- 
- [Building the Code](#building-the-cpp-project)
  - [Dependencies](#dependencies)
  - [Building](#building)
- [Usage](#usage)
  - [Calibration](#calibration)
  - [Thrust Vector Control](#thrust-vector-control)

## Building the cpp Project
### Dependencies

Please refer to `CMakeLists.txt`. 

### Building
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
### Calibration
:warning: Warning: Assure that appropriate safety routines are followed! This includes among others
- Having the rotor under test in a protected enclosure
- Wearing PPE(glasses, ear protection)
- Assuring the rotor is securely fastened and is not damaged

#### Configure
Configure the motor command sequence to be generated in `src/main_calibration.cpp`. 
#### Running
After configuring and building the project, you can run the calibration by executing the generated binary file. Please ensure that your Moteus motor driver, Raspberry Pi, and test stand setup are properly connected and configured before running the calibration. The Moteus controller should be connected to the "JC1" CAN port on the Pi3Hat. 
```
sudo ./build/main_calibration
```
#### Analyzing
To analyze the calibration, prepare recorded motor telemetry and force/torque measurements. If the force measurements are not synced, this must be performed manually. 
Use `scripts/plot_single_datase.py` to visualize force/torque measurements against motor telemetry with the generated sequences highlighted. Note the timeshift between this data. The force/torque csv file
should be updated with a header element `Force Time Offset = 0.0`. Update this with the manual timeshift. Running `scripts/plot_single_datase.py` again should now show synchronized data. 
#### Example results
<img src="https://user-images.githubusercontent.com/12870693/234284012-f81d746c-369f-4833-95ee-9fb075397dca.png" width="40%" height="40%"><img src="https://user-images.githubusercontent.com/12870693/234284536-6e1de7c5-f816-4678-b291-5b7fe1cc4ee6.png" width="40%" height="40%"><img src="https://user-images.githubusercontent.com/12870693/234284395-f2d7e31a-be34-46f9-950f-5182517e2069.png" width="40%" height="40%">
### Thrust Vector Control
**TODO**

Please note that this repository is a work in progress and does not have the ambition to be highly reusable.
