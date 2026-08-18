//
// Created by clemens on 31.07.24.
//

#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

#include <ImuServiceBase.hpp>


using namespace xbot::service;

class ImuService : public ImuServiceBase {
 public:
  explicit ImuService(const uint16_t service_id) : ImuServiceBase(service_id) {
  }

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 10'000,
                                 XBOT_FUNCTION_FOR_METHOD(ImuService, &ImuService::tick, this)};
};

#endif  // IMU_SERVICE_HPP
