// Minimal Charger driver interface (MCU-backed stub)
#pragma once

#include <cstdint>

namespace xbot::driver::charger {

class ChargerDriver {
 public:
  enum class ReChargeVoltage : uint8_t { PERCENT_93_0 = 0, PERCENT_94_3 = 1, PERCENT_95_2 = 2, PERCENT_97_6 = 3 };

  enum class CHARGER_STATUS : uint8_t { COMMS_ERROR = 0, IDLE = 1, CHARGING = 2, FULL = 3 };

  virtual ~ChargerDriver() = default;

  virtual bool init() = 0;
  virtual bool setPreChargeCurrent(float amp) = 0;
  virtual bool setChargingCurrent(float amp, bool overwrite_hardware_limit) = 0;
  virtual bool setChargingVoltage(float volts) = 0;
  virtual bool setTerminationCurrent(float amp) = 0;
  virtual bool setReChargeVoltage(ReChargeVoltage v) = 0;
  virtual bool setTsEnabled(bool enabled) = 0;

  virtual bool resetWatchdog() = 0;
  virtual bool readChargeCurrent(float &out) = 0;
  virtual bool readBatteryVoltage(float &out) = 0;
  virtual bool readAdapterVoltage(float &out) = 0;
  virtual bool readAdapterCurrent(float &out) = 0;

  virtual CHARGER_STATUS getChargerStatus() = 0;

  static const char* statusToString(CHARGER_STATUS s) {
    switch (s) {
      case CHARGER_STATUS::COMMS_ERROR: return "COMMS_ERROR";
      case CHARGER_STATUS::IDLE: return "IDLE";
      case CHARGER_STATUS::CHARGING: return "CHARGING";
      case CHARGER_STATUS::FULL: return "FULL";
    }
    return "UNKNOWN";
  }
};

} // namespace xbot::driver::charger
