#ifndef POWER_SERVICE_HPP
#define POWER_SERVICE_HPP
#include <drivers/power/power_driver.hpp>
#include <PowerServiceBase.hpp>

using namespace xbot::service;

class PowerService : public PowerServiceBase {
 private:
  xbot::driver::power::PowerDriver* driver_;

 public:
  explicit PowerService(uint16_t service_id) : PowerServiceBase(service_id) {
  }
  bool OnStart();

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 200'000,
                                 XBOT_FUNCTION_FOR_METHOD(PowerService, &PowerService::tick, this)};

};

#endif  // POWER_SERVICE_HPP
