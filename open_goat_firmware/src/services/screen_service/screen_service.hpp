#ifndef SCREEN_SERVICE_HPP
#define SCREEN_SERVICE_HPP
#include <cstdint>
#include <drivers/screen/screen_driver.hpp>
#include <ScreenServiceBase.hpp>

using namespace xbot::service;

class ScreenService : public ScreenServiceBase {
 private:
  xbot::driver::screen::ScreenDriver* driver_;

 public:
  explicit ScreenService(uint16_t service_id) : ScreenServiceBase(service_id) {
  }
  bool OnStart();

  bool IsRaining() const {
    if (driver_ == nullptr) {
      return false;
    }
    return driver_->IsRaining();
  }

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 200'000,
                                 XBOT_FUNCTION_FOR_METHOD(ScreenService, &ScreenService::tick, this)};

};

#endif  // SCREEN_SERVICE_HPP
