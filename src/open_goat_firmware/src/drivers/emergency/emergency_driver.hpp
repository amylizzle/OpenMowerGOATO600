// Minimal Emergency driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>

namespace xbot::driver::emergency {

class EmergencyDriver {
 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_; 

  uint8_t bump = 0;
  uint8_t fall = 0;
  uint8_t chargeState = 0;
  uint8_t acczero = 0;
  uint8_t rain = 0;
  uint8_t grass = 0;
  uint8_t roll = 0;
  uint8_t Stop = 0;
  uint8_t fan = 0;
  uint8_t ack = 0;

 public:
  EmergencyDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~EmergencyDriver() = default;

  // handle BC messages
  void OnBCMessage(const uint8_t *payload, size_t length);

  // Manually trigger/add or clear emergency bits via the driver
  void UpdateEmergency(uint16_t add, uint16_t clear);
};

} // namespace xbot::driver::emergency
