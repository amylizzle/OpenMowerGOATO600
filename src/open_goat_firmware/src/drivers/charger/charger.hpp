// Thin wrapper to MCU-backed Charger driver interface
#pragma once

#include "../mcu/charger.hpp"

namespace xbot::driver::charger {
using ::xbot::driver::charger::ChargerDriver;
using ::xbot::driver::charger::CHARGER_STATUS;
using ::xbot::driver::charger::ReChargeVoltage;
}
