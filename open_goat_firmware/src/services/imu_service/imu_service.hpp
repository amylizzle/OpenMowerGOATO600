#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

#include <drivers/imu/imu_driver.hpp>
#include <ImuServiceBase.hpp>

using namespace xbot::service;

class ImuService : public ImuServiceBase {
 private:
  xbot::driver::imu::ImuDriver* driver_;

 public:
  explicit ImuService(const uint16_t service_id) : ImuServiceBase(service_id) {
  }
 protected:
  bool OnStart() override;

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 10'000,
                                 XBOT_FUNCTION_FOR_METHOD(ImuService, &ImuService::tick, this)};
};

#endif  // IMU_SERVICE_HPP
