//
// Created by clemens on 31.07.24.
//

#ifndef MOWER_SERVICE_HPP
#define MOWER_SERVICE_HPP

#include <MowerServiceBase.hpp>

using namespace xbot::service;

class MowerService : public MowerServiceBase {
 public:
  explicit MowerService(const uint16_t service_id) : MowerServiceBase(service_id) {
  }

 private:
  bool mower_running_ = false;
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 1'000'000,
                                 XBOT_FUNCTION_FOR_METHOD(MowerService, &MowerService::tick, this)};

 protected:
  bool OnStart() override;
  void OnMowerSpeedChanged(const float& new_value);
};

#endif  // MOWER_SERVICE_HPP
