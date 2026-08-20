//
// Created by clemens on 26.07.24.
//

#include "diff_drive_service.hpp"

#include <globals.hpp>

void DiffDriveService::OnStop() {
  if (driver_ != nullptr) {
    driver_->SetDuty(0.0f,0.0f,0.0f);
  }
}

void DiffDriveService::tick() {
  if (driver_ == nullptr) {
    return;
  }

  const auto& left_state = driver_->GetLeftState();
  const auto& right_state = driver_->GetRightState();

  StartTransaction();
  SendLeftESCStatus(static_cast<uint8_t>(left_state.status == xbot::driver::motor::MotorDriver::ESCState::ESCStatus::ESC_STATUS_OK ? 200u : 0u));
  SendRightESCStatus(static_cast<uint8_t>(right_state.status == xbot::driver::motor::MotorDriver::ESCState::ESCStatus::ESC_STATUS_OK ? 200u : 0u));

  //wheel ticks per meter and separation distance from service parameters
  float wheel_radius = 1.0;
  float track_width = 0.3;

  double twist[6]{0};
  // 1. Convert RPM to wheel linear velocity (m/s)
  // v_wheel = RPM * (2 * PI / 60) * wheel_radius
  const float rpm_to_v_wheel = (2.0f * M_PI / 60.0f) * wheel_radius;

  float v_left  = left_state.rpm  * rpm_to_v_wheel;
  float v_right = right_state.rpm * rpm_to_v_wheel;

  // 2. Forward Kinematics (Wheel Speeds -> Robot Twist)
  twist[0]  = (v_right + v_left) / 2.0f;           // Average linear velocity
  twist[5] = (v_right - v_left) / track_width;     // Yaw rate (rad/s)
  SendActualTwist(twist, sizeof(twist) / sizeof(double));
  CommitTransaction();
}

void DiffDriveService::OnControlTwistChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  // data[0] = msg->linear.x;
  // data[1] = msg->linear.y;
  // data[2] = msg->linear.z;
  // data[3] = msg->angular.x;
  // data[4] = msg->angular.y;
  // data[5] = msg->angular.z;

  // we can only do forward and rotation around one axis
  const auto linear = static_cast<float>(new_value[0]);
  const auto angular = static_cast<float>(new_value[5]);

  // Optional scaling factors depending on your robot's max limits
  // Tune these if your raw command values exceed typical bounds
  float max_linear_vel = 1.0f;  // m/s
  float max_angular_vel = 2.0f; // rad/s

  // 1. Differential Drive Mixer
  // Linear velocity adds to both wheels equally; angular creates opposing wheel speeds.
  float left_raw = (linear / max_linear_vel) - (angular / max_angular_vel);
  float right_raw = (linear / max_linear_vel) + (angular / max_angular_vel);

  // 2. Normalization / Desaturation
  // If combined inputs exceed 1.0, scale both proportionally to maintain turning radius.
  float max_mag = std::max(std::abs(left_raw), std::abs(right_raw));

  float leftval = left_raw;
  float rightval = right_raw;

  if (max_mag > 1.0f) {
      leftval /= max_mag;
      rightval /= max_mag;
  }

  // Optional hard safety clamp [-1.0f, 1.0f]
  leftval = std::clamp(leftval, -1.0f, 1.0f);
  rightval = std::clamp(rightval, -1.0f, 1.0f);

  // Send normalized values to motor driver
  driver_->SetDuty(leftval, rightval, 0.0f);
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

xbot::driver::motor::MotorDriver* DiffDriveService::GetDriverInstance() {
  if (driver_ == nullptr) {
    driver_ = new xbot::driver::motor::MotorDriver(&mcu_dispatcher_driver);
  }
  return driver_; 
}