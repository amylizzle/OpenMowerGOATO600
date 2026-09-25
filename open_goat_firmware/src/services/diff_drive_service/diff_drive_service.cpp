#include "diff_drive_service.hpp"

#include <globals.hpp>
#include <xbot-service/portable/system.hpp>
#include <algorithm>
#include <cmath>

void DiffDriveService::OnStop() {
  if (driver_ != nullptr) {
    driver_->SetDuty(0.0f,0.0f,0.0f);
  }
}

void DiffDriveService::tick() {
  if (driver_ == nullptr) {
    return;
  }

  // Check, if we recently received duty. If not, set to zero for safety
  if (xbot::service::system::getTimeMicros() - last_duty_received_micros_ > 1'000'000) {
    driver_->SetDuty(0.0f,0.0f,0.0f); //include the mow motor, cos why not
  }

  const auto& left_state = driver_->GetLeftState();
  const auto& right_state = driver_->GetRightState();

  StartTransaction();
  SendLeftESCStatus(static_cast<uint8_t>(left_state.status == xbot::driver::motor::MotorDriver::ESCState::ESCStatus::ESC_STATUS_OK ? 200u : 0u));
  SendRightESCStatus(static_cast<uint8_t>(right_state.status == xbot::driver::motor::MotorDriver::ESCState::ESCStatus::ESC_STATUS_OK ? 200u : 0u));
  // add min int32 to avoid overflow when converting to uint32_t
  uint32_t wheelticks[2]{ static_cast<uint32_t>(left_state.tacho+0x80000000), static_cast<uint32_t>(right_state.tacho+0x80000000) };
  SendWheelTicks(wheelticks, sizeof(wheelticks) / sizeof(uint32_t));
  SendLeftESCCurrent(static_cast<float>(left_state.current_input));
  SendRightESCCurrent(static_cast<float>(right_state.current_input));
  SendLeftESCTemperature(static_cast<float>(left_state.temperature_pcb));
  SendRightESCTemperature(static_cast<float>(right_state.temperature_pcb));

  double twist[6]{0};
  uint32_t now_ms = chVTGetSystemTimeX();
  float dt = (now_ms - last_tick_time_ms_) * 1e-3f;

  float v_left  = 0.0f, v_right = 0.0f;
  if (dt > 0.0f) {
      v_left  = (left_state.tacho  - last_left_tacho_)  / WheelTicksPerMeter.value / dt;
      v_right = (right_state.tacho - last_right_tacho_) / WheelTicksPerMeter.value / dt;
  }

  last_left_tacho_  = left_state.tacho;
  last_right_tacho_ = right_state.tacho;
  last_tick_time_ms_ = now_ms;

  twist[0]  = (v_right + v_left) / 2.0f; // Average linear velocity
  twist[5] = (v_right - v_left) / this->WheelDistance.value; // Yaw rate (rad/s)
  SendActualTwist(twist, sizeof(twist) / sizeof(double));
  CommitTransaction();
}

void DiffDriveService::OnControlTwistChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  last_duty_received_micros_ = xbot::service::system::getTimeMicros();

  bool emergency = emergency_service.GetEmergencyReasons() != 0;
  if (emergency) {
    if (driver_ != nullptr) {
      driver_->SetDuty(0.0f,0.0f,0.0f);
    }
    return;
  }
  // data[0] = msg->linear.x;
  // data[1] = msg->linear.y;
  // data[2] = msg->linear.z;
  // data[3] = msg->angular.x;
  // data[4] = msg->angular.y;
  // data[5] = msg->angular.z;

  // we can only do forward and rotation around one axis
  const auto linear = static_cast<float>(new_value[0]); //m/s
  const auto angular = static_cast<float>(new_value[5]); //rad/s

  // Inverse kinematics: each wheel carries half the rotation over the track width
  const auto half_track = static_cast<float>(WheelDistance.value) / 2.0f;
  float leftval = linear - angular * half_track;   // m/s
  float rightval = linear + angular * half_track;  // m/s

  // Scale both wheels together if either saturates, so the commanded curvature
  // is kept instead of clipping one wheel and driving the wrong arc
  const float max_mag = std::max(std::abs(leftval), std::abs(rightval));
  if (max_mag > 1.0f) {
    leftval /= max_mag;
    rightval /= max_mag;
  }

  // Send normalized values to motor driver
  driver_->SetDuty(leftval, rightval, std::nullopt);
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

// called by emergency service
void DiffDriveService::OnEmergencyChangedEvent() {
  bool emergency = emergency_service.GetEmergencyReasons() != 0;
  if (!emergency) {
    // only set speed to 0 if the emergency happens, not if it's cleared
    return;
  }
  if (driver_ != nullptr) {
    driver_->SetDuty(0.0f,0.0f,0.0f);
  }
}

xbot::driver::motor::MotorDriver* DiffDriveService::GetDriverInstance() {
  if (driver_ == nullptr) {
    driver_ = new xbot::driver::motor::MotorDriver(&mcu_dispatcher_driver);
  }
  return driver_; 
}