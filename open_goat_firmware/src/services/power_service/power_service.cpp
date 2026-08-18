#include <string>
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
  SendBatteryVoltage(powerdata.stateOfCharge);
  SendChargeVoltage(powerdata.chargeVoltage / 1000.0f);
  SendChargeCurrent(powerdata.chargeCurrent / 1000.0f);
  SendChargerEnabled(powerdata.chargeVoltage > 0);
  // SendChargingStatus(charge_state.c_str(), charge_state.length());
  CommitTransaction();
}

