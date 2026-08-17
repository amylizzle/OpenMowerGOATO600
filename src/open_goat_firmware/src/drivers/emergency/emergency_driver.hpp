// Minimal Emergency driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>

namespace xbot::driver::emergency {

class EmergencyDriver {
 public:
  using EmergencyCallback = etl::delegate<void(uint16_t reasons)>;

  virtual ~EmergencyDriver() = default;

  // Start driver (e.g. open UART/SPI/etc)
  virtual bool Start() = 0;

  // Stop driver
  virtual void Stop() = 0;

  // Returns true if hardware is present
  virtual bool IsPresent() const = 0;

  // Set callback invoked when driver detects an emergency reason change
  virtual void SetCallback(const EmergencyCallback& cb) = 0;

  // Manually trigger/add or clear emergency bits via the driver
  virtual void UpdateEmergency(uint16_t add, uint16_t clear) = 0;
};

} // namespace xbot::driver::emergency
