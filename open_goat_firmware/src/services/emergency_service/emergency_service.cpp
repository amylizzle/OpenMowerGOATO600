//
// Created by clemens on 26.07.24.
//
#include <globals.hpp>
#include "emergency_service.hpp"

using namespace xbot::driver::emergency;
// How long we tolerate silence from the high level before latching TIMEOUT_HIGH_LEVEL.
// Matches the firmware's 1 s window.
static constexpr double HIGH_LEVEL_TIMEOUT_S = 1.0;

bool EmergencyService::OnStart() {

  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = new EmergencyDriver(&mcu_dispatcher_driver);
  }

  driver_->RegisterNotifyCallback(etl::delegate<void(const uint16_t)>::create<EmergencyService, &EmergencyService::OnDriverNotify>(*this));

  return true;
}

void EmergencyService::OnDriverNotify(const uint16_t emergencyState) {
  StartTransaction();
  SendEmergencyReason(emergencyState);
  CommitTransaction();
}

void EmergencyService::OnStop() {
  // We won't get further updates from the high level, so raise the timeout reason.
  // robot_.ApplyEmergencyUpdate(EmergencyReason::TIMEOUT_HIGH_LEVEL, 0);
}

void EmergencyService::OnHighLevelEmergencyChanged(const uint16_t* /* new_value */, uint32_t /* length*/) {
  // last_high_level_emergency_message_ = chVTGetSystemTimeX();
}

void EmergencyService::tick() {
  uint16_t emergency_reason = driver_->GetEmergencyState();

  StartTransaction();
  SendEmergencyReason(emergency_reason);
  CommitTransaction();
}
