#include <globals.hpp>
#include "emergency_service.hpp"
#include "posix_ch.h"
#include <etl/algorithm.h>
#include <xbot-service/portable/system.hpp>

using namespace xbot::driver::emergency;
// How long we tolerate silence from the high level before latching TIMEOUT_HIGH_LEVEL.
// Matches the firmware's 1 s window.
static constexpr double HIGH_LEVEL_TIMEOUT_S = 1.0;

bool EmergencyService::OnStart() {

  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = new EmergencyDriver(&mcu_dispatcher_driver);
  }
  reasons_ = 0;
  driver_->RegisterNotifyCallback(etl::delegate<void(const uint16_t)>::create<EmergencyService, &EmergencyService::OnDriverNotify>(*this));
  SendStatus();
  return true;
}

void EmergencyService::OnDriverNotify(const uint16_t emergencyState) {
  UpdateEmergency(emergencyState);
}


void EmergencyService::OnStop() {
  // We won't be getting further updates from high level, so set that flag immediately.
  UpdateEmergency(EmergencyReason::TIMEOUT_HIGH_LEVEL);
}

uint32_t EmergencyService::OnLoop(uint32_t now_micros, uint32_t) {
  return CheckTimeouts(now_micros);
}

void EmergencyService::OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) {
  (void)length;
  last_high_level_emergency_message_ = xbot::service::system::getTimeMicros();
  UpdateEmergency(new_value[0], new_value[1]);
}

uint32_t EmergencyService::CheckTimeouts(uint32_t now) {
  uint16_t reasons = 0;
  uint32_t block_time = UINT32_MAX;
  
  if (TimeoutReached(now - last_high_level_emergency_message_, 1'000'000, block_time)) {
    reasons |= EmergencyReason::TIMEOUT_HIGH_LEVEL;
  }

  constexpr uint16_t potential_reasons = EmergencyReason::TIMEOUT_HIGH_LEVEL | EmergencyReason::TIMEOUT_INPUTS;
  UpdateEmergency(reasons, potential_reasons);
  return block_time;
}

void EmergencyService::UpdateEmergency(uint16_t add, uint16_t clear) {
  uint16_t old_reason = reasons_;
  reasons_ &= ~clear;
  reasons_ |= add;
  if (reasons_ == old_reason) {
    return;
  }
  SendStatus();
}

uint16_t EmergencyService::GetEmergencyReasons() {
  return reasons_;
}

void EmergencyService::SendStatus() {
  StartTransaction();
  SendEmergencyReason(reasons_);
  CommitTransaction();
}

void EmergencyService::tick() {
  uint16_t emergency_reason = driver_->GetEmergencyState();

  UpdateEmergency(emergency_reason);
}
