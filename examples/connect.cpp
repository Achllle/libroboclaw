#include <iostream>

#include "libroboclaw/roboclaw_driver.h"
#include "libroboclawConfig.h"

int main(int argc, char *argv[]) {

  if (argc < 4) {
    // report version
    std::cout << argv[0] << " Version " << libroboclaw_VERSION_MAJOR << "."
              << libroboclaw_VERSION_MINOR << std::endl;
    std::cout << "Usage: " << argv[0] << " serial_port baudrate address" << std::endl;
    std::cout << "Example: " << argv[0] << " /dev/serial0 115200 128" << std::endl;
    return 1;
  }

  std::string serial_port = argv[1];
  unsigned int baudrate = std::stoi(argv[2]);
  unsigned int addr = std::stoi(argv[3]);
  std::cout << "attempting to establish connection to port " << serial_port 
            << " with baud rate " << baudrate << " at address " << addr
            << "..." << std::endl;
  libroboclaw::driver *roboclaw_connections;
  roboclaw_connections = new libroboclaw::driver(serial_port, baudrate);

  // get encoder values
  std::pair<int, int> enc_res = roboclaw_connections->get_encoders(addr);
  std::cout << "encoders: M1: " << enc_res.first << ", M2: " << enc_res.second << std::endl;

  // get velocity
  std::pair<int, int> vel_res = roboclaw_connections->get_velocity(addr);
  std::cout << "velocity values: M1: " << vel_res.first << ", M2: " << vel_res.second << std::endl;

  // get pwm
  std::pair<int, int> pwm_res = roboclaw_connections->get_pwm(addr);
  std::cout << "pwm values: M1: " << pwm_res.first << ", M2: " << pwm_res.second << std::endl;

  // get duty cycle
  std::pair<double, double> duty_res = roboclaw_connections->get_duty_cycle(addr);
  std::cout << "duty cycle: M1: " << duty_res.first << ", M2: " << duty_res.second << std::endl;

  // get current
  std::pair<int, int> curr_res = roboclaw_connections->get_current(addr);
  std::cout << "current: M1: " << curr_res.first << ", M2: " << curr_res.second << std::endl;

  // get error
  int error_res = roboclaw_connections->get_status(addr);
  std::cout << "error: " << error_res << std::endl;

  // get config
  int config = roboclaw_connections->get_config(addr);
  std::cout << "config: " << config << std::endl;

  // set config
  roboclaw_connections->set_config(addr, 8355);
  int new_config = roboclaw_connections->get_config(addr);
  std::cout << "new config: " << new_config << std::endl;

  // get max and min current
  std::pair<std::pair<int, int>, std::pair<int, int>> max_min_curr_res = roboclaw_connections->get_current_limits(addr);
  std::cout << "max current: M1: " << max_min_curr_res.first.first << ", M2: " << max_min_curr_res.second.first << std::endl;
  std::cout << "min current: M1: " << max_min_curr_res.first.second << ", M2: " << max_min_curr_res.second.second << std::endl;

  // get version (warning: brittle!)
  std::cout << "version: " << roboclaw_connections->get_version(addr) << std::endl;

  return 0;
}