#include "power_driver.hpp"
#include <drivers/mcu/dispatcher.hpp>
#include <etl/delegate.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <sstream>

namespace xbot::driver::power {

PowerDriver::PowerDriver(mcu::Dispatcher *dispatcher) {
  if (dispatcher) {
    dispatcher->RegisterHandler(
        static_cast<uint8_t>('C'), static_cast<uint8_t>('C'),
        etl::delegate<void(const uint8_t *, size_t)>::create<PowerDriver, &PowerDriver::OnMessage>(*this));
  }
}

// Handler called by Dispatcher when cmd0='C', cmd1='C' message arrives
void PowerDriver::OnMessage(const uint8_t *payload, size_t length) {
  static constexpr size_t EXPECTED_LEN = 9;
  if (length < EXPECTED_LEN) return;

  data_.stateOfCharge = payload[0];
  data_.chargingState = payload[1];
  data_.chargeVoltage = static_cast<uint16_t>(payload[2]) | (static_cast<uint16_t>(payload[3]) << 8);
  data_.chargeCurrent = static_cast<int16_t>(payload[4]) | (static_cast<int16_t>(payload[5]) << 8);
  data_.batteryTemp = static_cast<int8_t>(payload[6]);
  data_.batteryID = payload[7];
  data_.chargeStep = payload[8];

  // if (length > 0x13):
  //     data_.ext = [_u16(data, 9 + 2*k) for k in range(5)]
}

PowerDriver::Data PowerDriver::GetData() {
  return data_;
}

};
