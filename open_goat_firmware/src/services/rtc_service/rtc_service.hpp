#ifndef RTC_SERVICE_HPP
#define RTC_SERVICE_HPP

#include <drivers/rtc/rtc_driver.hpp>
#include <RTCServiceBase.hpp>

using namespace xbot::service;

class RTCService : public RTCServiceBase {
 private:
  xbot::driver::rtc::RTCDriver* driver_;

 public:
  explicit RTCService(const uint16_t service_id) : RTCServiceBase(service_id) {
  }
 protected:
  bool OnStart() override;

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 30'000'000,
                                 XBOT_FUNCTION_FOR_METHOD(RTCService, &RTCService::tick, this)};
};

#endif  // IMU_SERVICE_HPP
