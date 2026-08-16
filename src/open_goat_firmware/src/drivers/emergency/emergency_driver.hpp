// Thin wrapper to MCU-backed Emergency driver interface
#pragma once

#include "../mcu/emergency_driver.hpp"

namespace xbot::driver::emergency {
using ::xbot::driver::emergency::EmergencyDriver;
using ::xbot::driver::emergency::EmergencyCallback;
}
