// Minimal Emergency driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>

namespace xbot::driver::emergency {

class EmergencyDriver {
 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_; 

 public:
  ~EmergencyDriver() = default;

  // Start driver (e.g. open UART/SPI/etc)
  virtual bool Start() = 0;

  // Stop driver
  virtual void Stop() = 0;

  // Returns true if hardware is present
  virtual bool IsPresent() const = 0;

  // Manually trigger/add or clear emergency bits via the driver
  virtual void UpdateEmergency(uint16_t add, uint16_t clear) = 0;
};

} // namespace xbot::driver::emergency
