// Minimal BMS driver interface (MCU-backed stub)
#pragma once

#include <cstdint>

namespace xbot::driver::bms {

struct Data {
  float pack_voltage_v = 0.0f;
  float pack_current_a = 0.0f;
  float battery_soc = 0.0f;
  float remaining_capacity_ah = 0.0f;
  float full_charge_capacity_ah = 0.0f;
  uint32_t cycle_count = 0;
  float temperature_c = 0.0f;
  uint8_t battery_status = 0;
};

class BmsDriver {
 public:
  virtual ~BmsDriver() = default;

  // Called periodically by the service to update internal state
  virtual void Tick() = 0;

  // Returns true if a BMS is present/communicating
  virtual bool IsPresent() const = 0;

  // Returns pointer to latest data (owned by driver)
  virtual const Data* GetData() const = 0;

  // Returns optional extra data as JSON string (or nullptr)
  virtual const char* GetExtraDataJson() const = 0;
};

} // namespace xbot::driver::bms
