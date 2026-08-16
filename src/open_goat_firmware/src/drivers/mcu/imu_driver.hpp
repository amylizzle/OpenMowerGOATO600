// Minimal IMU driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>

namespace xbot::driver::imu {

class ImuDriver {
 public:
  using DataReadyCallback = etl::delegate<void()>;

  virtual ~ImuDriver() = default;

  // Initialize hardware / bus
  virtual bool Start() = 0;

  // Return true if device is present
  virtual bool IsPresent() const = 0;

  // Read latest axes data. Expect length==9 (3 accel, 3 gyro, 3 reserved) or similar.
  virtual bool ReadAxes(double* axes, size_t length) = 0;

  // Set a callback when new data is available
  virtual void SetDataReadyCallback(const DataReadyCallback& cb) = 0;
};

} // namespace xbot::driver::imu
