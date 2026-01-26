#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstring>

#include "libroboclaw/roboclaw_driver.h"
#include "libroboclawConfig.h"

int main(int argc, char *argv[]) {

  if (argc < 5) {
    // report version
    std::cout << argv[0] << " Version " << libroboclaw_VERSION_MAJOR << "."
              << libroboclaw_VERSION_MINOR << std::endl;
    std::cout << "Usage: " << argv[0] << " serial_port baudrate address motor [timeout] [duty]" << std::endl;
    std::cout << "  motor: M1 or M2" << std::endl;
    std::cout << "  timeout: timeout in seconds (default: 2.0, range: 0.1-25.5)" << std::endl;
    std::cout << "  duty: duty cycle percentage (default: 20, range: 1-100)" << std::endl;
    std::cout << "Example: " << argv[0] << " /dev/serial0 115200 128 M1 2.0 20" << std::endl;
    return 1;
  }

  std::string serial_port = argv[1];
  unsigned int baudrate = std::stoi(argv[2]);
  unsigned int addr = std::stoi(argv[3]);
  std::string motor = argv[4];
  float timeout = (argc > 5) ? std::stof(argv[5]) : 2.0f;
  int duty_percent = (argc > 6) ? std::stoi(argv[6]) : 20;

  // Validate motor argument
  if (motor != "M1" && motor != "M2") {
    std::cerr << "Error: motor must be M1 or M2" << std::endl;
    return 1;
  }

  // Validate timeout range
  if (timeout < 0.1f || timeout > 25.5f) {
    std::cerr << "Error: timeout must be between 0.1 and 25.5 seconds" << std::endl;
    return 1;
  }

  // Validate duty cycle
  if (duty_percent < 1 || duty_percent > 100) {
    std::cerr << "Error: duty must be between 1 and 100 percent" << std::endl;
    return 1;
  }

  // Convert duty percentage to raw value (32767 = 100%)
  int duty_raw = (int)(duty_percent * 32767.0 / 100.0);

  std::cout << "=== RoboClaw Serial Timeout Test ===" << std::endl;
  std::cout << "Port: " << serial_port << ", Baudrate: " << baudrate 
            << ", Address: " << addr << std::endl;
  std::cout << "Motor: " << motor << ", Timeout: " << timeout 
            << "s, Duty: " << duty_percent << "%" << std::endl;
  std::cout << std::endl;

  libroboclaw::driver *roboclaw = new libroboclaw::driver(serial_port, baudrate);

  // Step 1: Read and save original timeout value
  float original_timeout = roboclaw->get_serial_timeout(addr);
  std::cout << "Original timeout setting: " << original_timeout << "s" << std::endl;

  // Step 2: Set the new timeout
  std::cout << "Setting timeout to " << timeout << "s..." << std::endl;
  roboclaw->set_serial_timeout(addr, timeout);

  // Verify timeout was set
  float new_timeout = roboclaw->get_serial_timeout(addr);
  std::cout << "Timeout now set to: " << new_timeout << "s" << std::endl;

  // Step 3: Start the motor
  std::cout << "Starting " << motor << " at " << duty_percent << "% duty cycle..." << std::endl;
  if (motor == "M1") {
    roboclaw->set_duty(addr, std::make_pair(duty_raw, 0));
  } else {
    roboclaw->set_duty(addr, std::make_pair(0, duty_raw));
  }

  // Read initial duty cycle
  std::pair<double, double> duty_initial = roboclaw->get_duty_cycle(addr);
  double initial_duty = (motor == "M1") ? duty_initial.first : duty_initial.second;
  std::cout << "Initial duty cycle reading: " << initial_duty << "%" << std::endl;

  // Step 4: Wait for timeout + 0.5s margin
  float wait_time = timeout + 0.5f;
  std::cout << "Waiting " << wait_time << "s for timeout to trigger..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds((int)(wait_time * 1000)));

  // Step 5: Check if motor stopped
  std::pair<double, double> duty_after = roboclaw->get_duty_cycle(addr);
  double final_duty = (motor == "M1") ? duty_after.first : duty_after.second;
  std::cout << "Duty cycle after timeout: " << final_duty << "%" << std::endl;

  // Step 6: Validate result (5% threshold)
  const double DUTY_THRESHOLD = 5.0;
  bool timeout_worked = std::abs(final_duty) < DUTY_THRESHOLD;

  std::cout << std::endl;
  if (timeout_worked) {
    std::cout << "✓ SUCCESS: Timeout worked! Motor stopped automatically." << std::endl;
  } else {
    std::cout << "✗ FAILURE: Timeout did not stop the motor." << std::endl;
    std::cout << "  Manually stopping motor..." << std::endl;
    roboclaw->set_duty(addr, std::make_pair(0, 0));
    std::cout << "  Motor stopped manually." << std::endl;
  }

  // Step 7: Restore original timeout
  std::cout << std::endl;
  std::cout << "Restoring original timeout (" << original_timeout << "s)..." << std::endl;
  roboclaw->set_serial_timeout(addr, original_timeout);
  float restored_timeout = roboclaw->get_serial_timeout(addr);
  std::cout << "Timeout restored to: " << restored_timeout << "s" << std::endl;

  delete roboclaw;

  std::cout << std::endl;
  std::cout << "=== Test Complete ===" << std::endl;

  return timeout_worked ? 0 : 1;
}
