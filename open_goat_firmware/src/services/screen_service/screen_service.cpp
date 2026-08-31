#include <cstdint>
#include <globals.hpp>
#include "screen_service.hpp"

using namespace xbot::driver::screen;

bool ScreenService::OnStart() {

  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = new ScreenDriver(&mcu_dispatcher_driver);
  }

  return true;
}

void ScreenService::OnDriverNotify(const uint8_t eventCode) {
  if (eventCode == 6){
    //reset code, clear latched and stop
    emergency_service.UpdateEmergency(0, EmergencyReason::STOP | EmergencyReason::LATCH);
  }
}

void ScreenService::tick() {
  if (driver_ == nullptr) {
    return;
  }

  auto emergency = emergency_service.GetEmergencyReasons();
  if (emergency > 0){
    if (emergency & EmergencyReason::STOP) {
      driver_->SetScreenPage(ScreenDriver::ScreenPage::STOP);
    } else {
      driver_->SetErrorCode(emergency);
    }
  } else {
    driver_->SetScreenPage(ScreenDriver::ScreenPage::BATTERY);
  }
  auto gpsstate = gps_service.GetGpsState();
  switch(gpsstate.rtk_type){
    case GpsDriver::GpsState::RTK_NONE:
      driver_->SetInternetIcon(ScreenDriver::ScreenIconState::ICON_OFF);
      break;
    case GpsDriver::GpsState::RTK_FLOAT:
      driver_->SetInternetIcon(ScreenDriver::ScreenIconState::ICON_FLASH);
      break;
    case GpsDriver::GpsState::RTK_FIX:
      driver_->SetInternetIcon(ScreenDriver::ScreenIconState::ICON_ON);
      break;
  }

  //TODO get wifi state, and probably also generally manage wifi

}
