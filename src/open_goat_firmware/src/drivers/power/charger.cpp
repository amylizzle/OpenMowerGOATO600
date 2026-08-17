#include "charger.hpp"
#include "../mcu/dispatcher.h"

#include <etl/delegate.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <sstream>

namespace xbot::driver::charger {

class McuChargerDriver : public ChargerDriver {
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
    if (payload[length]==1) //placeholder variable usage
      return;
  }

  // virtual ~ChargerDriver() = default;
  // virtual bool setAdapterCurrent(float current_amps) = 0;
  // virtual bool setChargingCurrent(float current_amps, bool overwrite_hardware_limit) = 0;
  // virtual bool setChargingVoltage(float voltage_v) {
  //   (void)voltage_v;
  //   return true;
  // }
  // virtual bool setPreChargeCurrent(float current_amps) = 0;
  // virtual bool setTerminationCurrent(float current_amps) = 0;
  // virtual CHARGER_STATUS getChargerStatus() = 0;
  // virtual bool init() = 0;
  // virtual bool resetWatchdog() = 0;
  // virtual bool setTsEnabled(bool enabled) = 0;

  // virtual bool setReChargeVoltage(ReChargeVoltage recharge_voltage) {
  //   (void)recharge_voltage;
  //   return true;
  // }

  // virtual bool readChargeCurrent(float &result) = 0;
  // virtual bool readAdapterVoltage(float &result) = 0;
  // virtual bool readAdapterCurrent(float &result) = 0;
  // virtual bool readBatteryVoltage(float &result) = 0;

  // virtual float getChargeVoltageTarget() const {
  //   return std::numeric_limits<float>::quiet_NaN();
  // }

  // static constexpr const char *statusToString(CHARGER_STATUS status) {
  //   return CHARGER_STATUS_STRINGS[static_cast<size_t>(status)];
  // }
};
}