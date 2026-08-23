//
// Created by clemens on 26.07.24.
//

#ifndef EMERGENCY_SERVICE_HPP
#define EMERGENCY_SERVICE_HPP
#include <drivers/emergency/emergency_driver.hpp>
#include <EmergencyServiceBase.hpp>

using namespace xbot::service;

class EmergencyService : public EmergencyServiceBase {
 private:
  xbot::driver::emergency::EmergencyDriver* driver_;
 public:
  explicit EmergencyService(uint16_t service_id) : EmergencyServiceBase(service_id) {
  }
  uint16_t GetEmergencyReasons();

 protected:
  void OnStop() override;
  uint32_t OnLoop(uint32_t now_micros, uint32_t last_tick_micros) override;
  bool OnStart() override;
  void OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) override;
  void OnDriverNotify(const uint16_t emergencyState);

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 100'000,
                                 XBOT_FUNCTION_FOR_METHOD(EmergencyService, &EmergencyService::tick, this)};

  void UpdateEmergency(uint16_t add, uint16_t clear = 0);
  void SendStatus();
  uint32_t CheckTimeouts(uint32_t now);
  uint16_t reasons_ = EmergencyReason::TIMEOUT_INPUTS | EmergencyReason::TIMEOUT_HIGH_LEVEL;
  uint32_t last_high_level_emergency_message_ = 0;

};

#endif  // EMERGENCY_SERVICE_HPP
