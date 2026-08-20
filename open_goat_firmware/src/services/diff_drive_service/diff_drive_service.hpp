//
// Created by clemens on 26.07.24.
//

#ifndef DIFF_DRIVE_SERVICE_HPP
#define DIFF_DRIVE_SERVICE_HPP

#include <DiffDriveServiceBase.hpp>
#include <drivers/motor/motor_driver.hpp>

using namespace xbot::service;

class DiffDriveService : public DiffDriveServiceBase {
 public:
  explicit DiffDriveService(uint16_t service_id) : DiffDriveServiceBase(service_id) {
  }

  void OnMowerStatusChanged(uint32_t new_status);

 protected:
  bool OnStart() override;
  void OnStop() override;

 private:
  xbot::driver::motor::MotorDriver* driver_ = nullptr;

  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 20'000,
                                 XBOT_FUNCTION_FOR_METHOD(DiffDriveService, &DiffDriveService::tick, this)};
  void SetDuty();
  void ProcessStatusUpdate();

 protected:
  void OnControlTwistChanged(const double* new_value, uint32_t length) override;
};

#endif  // DIFF_DRIVE_SERVICE_HPP
