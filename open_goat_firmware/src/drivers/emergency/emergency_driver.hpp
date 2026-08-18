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
  using NotifyHandler = etl::delegate<void(const uint16_t emergencyState)>;
  etl::delegate<void(const uint16_t)> registered_handler_{};

  EmergencyDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~EmergencyDriver() = default;

  // handle BC messages
  void OnBCMessage(const uint8_t *payload, size_t length);

  // register a callback to be called when something changes
  // register a callback to be called when something changes
  // Accept the delegate by const-ref so callers can pass temporaries
  // and the driver will make its own copy for storage.
  void RegisterNotifyCallback(const NotifyHandler& handler);

  // calculate and return the emergency state
  uint16_t GetEmergencyState();

  // Manually trigger/add or clear emergency bits via the driver
  void UpdateEmergency(uint16_t add, uint16_t clear);
};

} // namespace xbot::driver::emergency
