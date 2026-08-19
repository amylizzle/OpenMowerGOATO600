#include <string>
#include <cstdint>
#include <globals.hpp>
#include "power_service.hpp"

using namespace xbot::driver::power;

bool PowerService::OnStart() {

  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = new PowerDriver(&mcu_dispatcher_driver);
  }

  return true;
}

void PowerService::tick() {
  PowerDriver::Data powerdata = driver_->GetData();
  // powerdata.chargingState;
  // powerdata.batteryTemp;
  // powerdata.batteryID;
  // powerdata.chargeStep;

  // Send the sensor values
  StartTransaction();
  SendBatteryVoltage(powerdata.batteryVoltage / 1000.0f);
  SendBatteryPercentage(powerdata.stateOfCharge);
  SendChargeCurrent(powerdata.batteryCurrent / 1000.0f);
  SendChargerEnabled(powerdata.batteryCurrent > 0);

  std::string charge_state = "cS " + std::to_string(+powerdata.chargingState) +
                           " bT " + std::to_string(+powerdata.batteryTemp) +
                           " ID " + std::to_string(+powerdata.batteryID) +
                           " st " + std::to_string(+powerdata.chargeStep);
  SendChargingStatus(charge_state.c_str(), charge_state.length());
  CommitTransaction();
}

