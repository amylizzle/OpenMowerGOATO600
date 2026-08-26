// Minimal Emergency driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>

namespace xbot::driver::emergency {

// ECOVACS UrgentAlarm DB payload (see hardware_control_RE/findings.md §10).
// mcuAlarmCode is a 32-bit bitmask; each set bit is one EMERGENCY_* condition.
enum EmergencyAlarmBit : uint32_t {
  EMERGENCY_INCLINE     = 1u << 0,
  EMERGENCY_ELEVATE     = 1u << 1,
  EMERGENCY_TURNOVER    = 1u << 2,
  EMERGENCY_BUMP        = 1u << 3,
  EMERGENCY_BACK        = 1u << 4,
  EMERGENCY_STOP        = 1u << 5,
  EMERGENCY_LSPEED      = 1u << 6,
  EMERGENCY_RSPEED      = 1u << 7,
  EMERGENCY_LWERR       = 1u << 8,
  EMERGENCY_RWERR       = 1u << 9,
  EMERGENCY_BATTEMP     = 1u << 10,
  EMERGENCY_HEART       = 1u << 11,
  EMERGENCY_GYRO        = 1u << 12,
  EMERGENCY_CORE_P      = 1u << 13,
  EMERGENCY_M12_P       = 1u << 14,
  EMERGENCY_LCERR       = 1u << 15,
  EMERGENCY_RCERR       = 1u << 16,
  EMERGENCY_LIFTMOT     = 1u << 17,
  EMERGENCY_BUMP_D      = 1u << 18,
  EMERGENCY_ELEVATE_D   = 1u << 19,
  EMERGENCY_CHARGEERR   = 1u << 20,
  EMERGENCY_GRASS_MOT   = 1u << 21,
  EMERGENCY_FANERR      = 1u << 22,
  EMERGENCY_INCLINE_D   = 1u << 23,
  EMERGENCY_BATERR      = 1u << 24,
};

// motorFaultCode holds 4 u8 (l_motor, r_motor, l_cut, r_cut); each byte is a
// MOTOR_FAULT_* code.
enum MotorFaultCode : uint8_t {
  MOTOR_FAULT_NONE = 0,
  MOTOR_FAULT_PHASE_LOSS = 1,
  MOTOR_FAULT_HARDWARE_OVER_CURRENT = 2,
  MOTOR_FAULT_SOFTWARE_OVER_CURRENT = 3,
  MOTOR_FAULT_STALL = 4,
  MOTOR_FAULT_HALL_SENSOR = 5,
  MOTOR_FAULT_UNDER_VOLTAGE = 6,
  MOTOR_FAULT_OVER_TEMPERATURE = 7,
  MOTOR_FAULT_POWER_INTEGRAL_LIMIT = 8,
};

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

  // DB (UrgentAlarm) raw state
  uint32_t mcuAlarmCode = 0;    // EMERGENCY_* bitmask
  uint32_t motorFaultCode = 0;  // 4 u8 motor err codes
  uint16_t liftFaultCode = 0;
  uint16_t grassFaultCode = 0;

  // Map the ECOVACS mcuAlarmCode EMERGENCY_* bits onto the host
  // EmergencyReason bitmask.
  static uint16_t AlarmBitsToEmergencyReason(uint32_t mcuAlarmCode);

 public:
  using NotifyHandler = etl::delegate<void(const uint16_t emergencyState)>;
  etl::delegate<void(const uint16_t)> registered_handler_{};

  EmergencyDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~EmergencyDriver() = default;

  // handle BC (GPIO sensor status) messages
  void OnBCMessage(const uint8_t *payload, size_t length, uint8_t ack);

  // handle DB (UrgentAlarm / MCU alert report) messages
  void OnDBMessage(const uint8_t *payload, size_t length, uint8_t ack);

  // register a callback to be called when something changes
  // Accept the delegate by const-ref so callers can pass temporaries
  // and the driver will make its own copy for storage.
  void RegisterNotifyCallback(const NotifyHandler& handler);

  // calculate and return the emergency state
  uint16_t GetEmergencyState();

  // Reset estop
  void ClearEStop();
};

} // namespace xbot::driver::emergency
