//
// Created by clemens on 02.04.25.
//

#ifndef CHARGER_HPP
#define CHARGER_HPP

#include <posix_ch.h>
#include <string>
#include <limits>
#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>

namespace xbot::driver::power {

class PowerDriver {
 public:
  struct Data {
    uint8_t stateOfCharge = 0; // %
    uint8_t chargingState = 0; // ????
    uint16_t chargeVoltage = 0.0f; //mV
    int16_t chargeCurrent = 0.0f; //mA
    int8_t batteryTemp = 0; //C
    uint8_t batteryID = 0; // ????
    uint8_t chargeStep = 0; // ????
  };

  PowerDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~PowerDriver() = default;
  Data GetData();

 private:
  mutable Data data_{};
  void OnMessage(const uint8_t *payload, size_t length);
};
};
#endif  // CHARGER_HPP