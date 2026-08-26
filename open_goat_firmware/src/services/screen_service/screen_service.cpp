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

void ScreenService::tick() {
  if (driver_ == nullptr) {
    return;
  }

  auto emergency = emergency_service.GetEmergencyReasons();
  if (emergency > 0){
    driver_->SetErrorCode(emergency);
  } else {
    driver_->SetScreenPage(xbot::driver::screen::ScreenDriver::ScreenPage::BATTERY);
  }
  auto gpsstate = gps_service.GetGpsState();
  switch(gpsstate.rtk_type){
    case xbot::driver::gps::GpsDriver::GpsState::RTK_NONE:
      driver_->SetInternetIcon(xbot::driver::screen::ScreenDriver::ScreenIconState::ICON_OFF);
      break;
    case xbot::driver::gps::GpsDriver::GpsState::RTK_FLOAT:
      driver_->SetInternetIcon(xbot::driver::screen::ScreenDriver::ScreenIconState::ICON_FLASH);
      break;
    case xbot::driver::gps::GpsDriver::GpsState::RTK_FIX:
      driver_->SetInternetIcon(xbot::driver::screen::ScreenDriver::ScreenIconState::ICON_ON);
      break;
  }

  //TODO get wifi state, and probably also generally manage wifi

}
