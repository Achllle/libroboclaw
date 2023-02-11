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

  // get pwm
  std::pair<int, int> pwm_res = roboclaw_connections->get_pwm(addr);
  std::cout << "pwm values: M1: " << pwm_res.first << ", M2: " << pwm_res.second << std::endl;

  // get duty cycle
  std::pair<double, double> duty_res = roboclaw_connections->get_duty_cycle(addr);
  std::cout << "duty cycle: M1: " << duty_res.first << ", M2: " << duty_res.second << std::endl;

  // get version (warning: likely brittle!)
  std::cout << roboclaw_connections->get_version(addr) << std::endl;

  

  return 0;
}