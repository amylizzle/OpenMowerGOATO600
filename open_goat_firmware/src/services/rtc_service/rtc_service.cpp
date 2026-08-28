#include <globals.hpp>
#include "rtc_service.hpp"

using namespace xbot::driver::rtc;

bool RTCService::OnStart() {

  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = new RTCDriver(&mcu_dispatcher_driver);
    driver_->Start();
  }

  return true;
}

void RTCService::tick() {
  driver_->Sync();
}
