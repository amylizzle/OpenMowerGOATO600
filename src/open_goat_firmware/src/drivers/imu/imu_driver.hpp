// Thin wrapper to MCU-backed IMU driver interface
#pragma once

#include "../mcu/imu_driver.hpp"

namespace xbot::driver::imu {
using ::xbot::driver::imu::ImuDriver;
using ::xbot::driver::imu::DataReadyCallback;
}
