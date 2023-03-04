#include <moteusapi/MoteusAPI.h>
#include <chrono>

int main() {
  // replace /dev/tty.usbmodemBE6118CD1 with your own usbcan dev name
  string dev_name("/dev/ttyACM0");
  int moteus_id = 1;
  MoteusAPI api(dev_name, moteus_id);

  // send one position with speed and torque limits
  double stop_position = NAN;
  double velocity = 2;
  double sinusoidal_amplitude = 0.5;
  double sinusoidal_phase = 0.0;
  double max_torque = 0.2;
  double feedforward_torque = 0;
  
  
  
  State curr_state;
  while (true) {
    auto start = std::chrono::high_resolution_clock::now();
    //api.ReadState(curr_state.EN_Velocity());
    api.SendSinusoidalPositionCommand(stop_position, velocity, max_torque, sinusoidal_amplitude, sinusoidal_phase,
                          feedforward_torque);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<float>>(stop-start);
    cout << "Velocity: " << curr_state.velocity << endl;
    cout << "Rate: " << 1/duration.count() << endl;
  }
  return 0;
}
