#include <cstdint>
#include <globals.hpp>
#include "screen_service.hpp"

using namespace xbot::driver::screen;

// Marker values displayed on the screen's err-code fields (little-endian u16).
static constexpr uint8_t DEAD_LOW = 0xAD;   // 0xDEAD low byte
static constexpr uint8_t DEAD_HIGH = 0xDE;  // 0xDEAD high byte
static constexpr uint8_t BEEF_LOW = 0xEF;   // 0xBEEF low byte
static constexpr uint8_t BEEF_HIGH = 0xBE;  // 0xBEEF high byte

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

  // Alternately display the battery percentage, "DEAD" and "BEEF".
  switch (display_phase_) {
    case 0:  // Battery percentage
      driver_->SetScreenState(
          /*lock=*/0, /*sim=*/0, /*wifi=*/0,
          /*page_num=*/power_service.GetBatteryPercentage(),
          /*err_low=*/0, /*err_high=*/0);
      break;
    case 1:  // "DEAD" (0xDEAD)
      driver_->SetScreenState(
          /*lock=*/0, /*sim=*/0, /*wifi=*/0,
          /*page_num=*/0,
          /*err_low=*/DEAD_LOW, /*err_high=*/DEAD_HIGH);
      break;
    case 2:  // "BEEF" (0xBEEF)
    default:
      driver_->SetScreenState(
          /*lock=*/0, /*sim=*/0, /*wifi=*/0,
          /*page_num=*/0,
          /*err_low=*/BEEF_LOW, /*err_high=*/BEEF_HIGH);
      break;
  }

  // Publish the currently displayed phase to the host.
  StartTransaction();
  SendDisplayState(display_phase_);
  CommitTransaction();

  display_phase_ = (display_phase_ + 1) % 3;
}
