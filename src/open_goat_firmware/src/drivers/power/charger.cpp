#include "charger.hpp"
#include <drivers/mcu/dispatcher.hpp>
#include <etl/delegate.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <sstream>

namespace xbot::driver::charger {

class McuChargerDriver : public ChargerDriver {
  mutable Data data_{};

 public:
  explicit McuChargerDriver(mcu::Dispatcher *dispatcher) {
    if (dispatcher) {
      dispatcher->RegisterHandler(
          static_cast<uint8_t>('C'), static_cast<uint8_t>('S'),
          etl::delegate<void(const uint8_t *, size_t)>::create<McuChargerDriver, &McuChargerDriver::OnMessage>(*this));
    }
  }

  private:
  // Handler called by Dispatcher when cmd0='C', cmd1='S' message arrives
  void OnMessage(const uint8_t *payload, size_t length) {
     // Format: '<BxHhbxB' -> little-endian:
    // [0] B             -> chargeStep (uint8)
    // [1] x             -> pad
    // [2..3] H          -> chargeVol (uint16)
    // [4..5] h          -> chargeCur (int16)
    // [6] b             -> batteryTemp (int8)
    // [7] x             -> pad
    // [8] B             -> batteryLevel (uint8)
    static constexpr size_t EXPECTED_LEN = 9;
    if (length < EXPECTED_LEN) return;

    uint8_t chargeStep = payload[0];
    uint16_t chargeVol = static_cast<uint16_t>(payload[2]) | (static_cast<uint16_t>(payload[3]) << 8);
    int16_t chargeCur = static_cast<int16_t>(static_cast<uint16_t>(payload[4]) | (static_cast<uint16_t>(payload[5]) << 8));
    int8_t batteryTemp = static_cast<int8_t>(payload[6]);
    uint8_t batteryLevel = payload[8];

    // Update Data (minimal / best-effort mapping)
    // pack_voltage_v: assume centivolts -> volts (user can adjust if needed)
    data_.pack_voltage_v = static_cast<float>(chargeVol) / 100.0f;
    // pack_current_a: assume centiamps -> amps
    data_.pack_current_a = static_cast<float>(chargeCur) / 100.0f;
    data_.temperature_c = static_cast<float>(batteryTemp);
    data_.battery_status = batteryLevel;
    data_.battery_soc = static_cast<float>(batteryLevel) / 100.0f;

    // Build simple JSON with raw parsed fields for GetExtraDataJson()
    std::ostringstream ss;
    ss << "{\"chargeStep\":" << static_cast<int>(chargeStep)
       << ",\"chargeVol\":" << chargeVol
       << ",\"chargeCur\":" << chargeCur
       << ",\"batteryTemp\":" << static_cast<int>(batteryTemp)
       << ",\"batteryLevel\":" << static_cast<int>(batteryLevel) << "}";
    data_.extra_json = ss.str();
  }
};
}