// Minimal Screen driver interface (MCU-backed stub)
//
// Implements the ECOVACS ScreenInfo protocol from hardware_control_RE/findings.md
// section 9. The screen MCU talks over the shared 0x60-framed link with two-char
// command ids starting with 'Z' (ZC/ZE/ZR/ZT receive, ZA send) plus CI (power mode).
//
// Wire frame: [0x60][ack][len=data+2][cmd0][cmd1][data...][crc8][0x0A]
#pragma once

#include <cstdint>
#include <vector>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>

namespace xbot::driver::screen {

class ScreenDriver {
 public:
  enum ScreenPowerState : uint8_t {
    POWER_OFF = 0,
    POWER_NORMAL = 1,
    POWER_SAVE = 2
  };

  enum ScreenIconState: uint8_t {
    ICON_OFF = 0,
    ICON_ON = 1,
    ICON_FLASH = 2
  };

  enum ScreenPage : uint8_t {
    SPIN = 0, //loading wheel (default state)
    PIN_INPUT = 1, //pin input mode
    BLANK = 2, // 4 dashes
    BLANK_DUP = 3, //not sure what this is for
    ON = 4, // "On" - if there's an "Off", I don't see it
    ON_DUP = 5, //nor this
    OTA = 6, //"ota"
    STOP = 7, //"STOP"
    ERROR = 8, //"E<num>" where num is decimal of err_high << 8 + err_low
    BATTERY = 9, //live battery percentage
  };

  // Mirrors the ScreenInfo state kept by the original node (obj+0x135..0x13c).
  struct ScreenState {
    ScreenIconState screen_lock = ScreenIconState::ICON_OFF;      // [0] screen lock
    ScreenIconState internet = ScreenIconState::ICON_OFF;         // [1] internet
    ScreenIconState wifi = ScreenIconState::ICON_OFF;             // [2] wifi
    ScreenPage page_num = ScreenPage::SPIN;                  // [3] page_num
    uint8_t err_code_low = 0;                                // [4] err_code_low
    uint8_t err_code_high = 0;                               // [5] err_code_high
    uint8_t pincode_confirm = 0;                             // [6] set to 1 for correct pin, 2 for incorrect pin - not really clear what that does
    uint8_t pincode_first = 0;                               // [7] unknown - maybe "this is the first pincode attempt"?
    // Derived receive state
    uint8_t rain = 0;             // ZR rain detect (1 = raining)
    uint16_t unlock_code = 0;     // ZC screen unlock code (16-bit signed)
    ScreenPowerState power_mode = ScreenPowerState::POWER_OFF;       // CI power mode 
  };

  // Optional callback for touch/control events from the screen (ZT).
  using NotifyHandler = etl::delegate<void(const uint8_t touch_code)>;

  ScreenDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~ScreenDriver() = default;

  // Send the current screen state (ZA) to the screen MCU.
  void Start();

  // ZA send: 8 bytes [lock][internet][wifi][page_num][err_low][err_high][pincode_confirm][pincode_first].
  void SetScreenState(ScreenIconState lock, ScreenIconState internet, ScreenIconState wifi, ScreenPage page_num,
                      uint8_t err_low, uint8_t err_high, uint8_t pincode_confirm = 0, uint8_t pincode_first = 0);

  // CI send: screen power mode (1-byte payload).
  void SetPowerMode(ScreenPowerState mode);

  // Register a callback to be called when the screen sends a touch/control event.
  void RegisterNotifyCallback(const NotifyHandler& handler);

  void SetLockIcon(ScreenIconState value);
  void SetInternetIcon(ScreenIconState value);
  void SetWifiIcon(ScreenIconState value);
  void SetScreenPage(ScreenPage value);
  void SetErrorCode(uint16_t num, bool show_error_page=true); //show_error_page: switch to error page if you're not already there

 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_{};
  ScreenState state_;
  NotifyHandler registered_handler_{};

  static inline int16_t ReadI16Le(const uint8_t* data, size_t offset);

  std::vector<uint8_t> EncodeScreenStateCommand();
  std::vector<uint8_t> EncodePowerModeCommand(ScreenPowerState mode);

  // Z? receive handlers (all dispatched by the 'Z' main command)
  void OnZC(const uint8_t* payload, size_t length, uint8_t ack);  // screen unlock code
  void OnZE(const uint8_t* payload, size_t length, uint8_t ack);  // key reset / factory reset
  void OnZR(const uint8_t* payload, size_t length, uint8_t ack);  // rain detect
  void OnZT(const uint8_t* payload, size_t length, uint8_t ack);  // touch/test/control
  void OnCI(const uint8_t* payload, size_t length, uint8_t ack);  // power mode request
};

}  // namespace xbot::driver::screen
