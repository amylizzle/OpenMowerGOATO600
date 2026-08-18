// Minimal IMU driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>


namespace xbot::driver::imu {

class ImuDriver {
 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_; 

 public:
  ImuDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~ImuDriver() = default;

  // Initialize hardware / bus
  void Start();

  // Read latest axes data. Expect length==9 (3 accel, 3 gyro, 3 reserved) or similar.
  void ReadAxes(double* axes, size_t length);
};

} // namespace xbot::driver::imu
