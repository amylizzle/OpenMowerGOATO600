//
// Created by clemens on 26.07.24.
//

#include "diff_drive_service.hpp"

#include <globals.hpp>

void DiffDriveService::OnStop() {
  if (driver_ != nullptr) {
    driver_->SetDuty(0.0f);
  }
}

void DiffDriveService::tick() {
  if (driver_ == nullptr) {
    return;
  }

  const auto& state = driver_->GetState();

  StartTransaction();
  SendLeftESCStatus(static_cast<uint8_t>(state.status == xbot::driver::motor::MotorDriver::ESCState::ESCStatus::ESC_STATUS_OK ? 200u : 0u));
  SendRightESCStatus(static_cast<uint8_t>(state.status == xbot::driver::motor::MotorDriver::ESCState::ESCStatus::ESC_STATUS_OK ? 200u : 0u));

  double twist[6]{0};
  twist[0] = static_cast<double>(state.rpm) / 1000.0;
  twist[5] = static_cast<double>(state.direction);
  SendActualTwist(twist, sizeof(twist) / sizeof(double));
  CommitTransaction();
}

void DiffDriveService::OnControlTwistChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  // we can only do forward and rotation around one axis
  const auto linear = static_cast<float>(new_value[0]);
  const auto angular = static_cast<float>(new_value[5]);

  // Translate the requested twist into a single duty value. The current MCU protocol
  // exposes a single scalar duty command for the mower drive, so this keeps the
  // service compatible with the existing driver abstraction.
  const float duty = std::clamp(linear + angular, -1.0f, 1.0f);
  driver_->SetDuty(duty);
}

bool DiffDriveService::OnStart() {
  if (driver_ == nullptr) {
    driver_ = new xbot::driver::motor::MotorDriver(&mcu_dispatcher_driver);
  }

  if (WheelDistance.value == 0 || WheelTicksPerMeter.value == 0.0) {
    return false;
  }

  driver_->Start();
  return true;
}
