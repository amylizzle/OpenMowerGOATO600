//
// Created by clemens on 31.07.24.
//

#include "mower_service.hpp"

#include <globals.hpp>

bool MowerService::OnStart() {
  if (driver_ == nullptr) {
    driver_ = diff_drive.GetDriverInstance(); //can't really have multiple instances of the same driver, use the diff drive's copy
  }

  driver_->Start();
  mower_running_ = false;
  return true;
}

void MowerService::tick() {
  const auto& state = driver_->GetMowState();

  StartTransaction();
  SendMowerRunning(state.rpm > 0);
  SendRainDetected(false);
  SendMowerMotorCurrent(static_cast<double>(state.current_input));
  SendMowerMotorRPM(static_cast<double>(state.rpm));
  SendMowerStatus(static_cast<uint16_t>(state.status == xbot::driver::motor::MotorDriver::ESCState::ESCStatus::ESC_STATUS_OK ? 200u : 0u));
  SendMowerMotorTemperature(static_cast<double>(state.temperature_motor));
  SendMowerESCTemperature(static_cast<double>(state.temperature_pcb));
  CommitTransaction();
}

void MowerService::OnMowerSpeedChanged(const float& new_value) {
  mower_running_ = new_value != 0.0f;
  if (driver_ != nullptr) {
    driver_->SetDuty(0.0f, 0.0f, mower_running_ ? new_value : 0.0f);
  }
}
