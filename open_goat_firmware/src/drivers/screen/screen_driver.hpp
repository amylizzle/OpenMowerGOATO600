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
  // Mirrors the ScreenInfo state kept by the original node (obj+0x135..0x13c).
  struct ScreenState {
    uint8_t screen_lock = 0;      // [0] screen lock
    uint8_t sim = 0;              // [1] sim
    uint8_t wifi = 0;             // [2] wifi
    uint8_t page_num = 0;         // [3] page_num
    uint8_t err_code_low = 0;     // [4] err_code_low
    uint8_t err_code_high = 0;    // [5] err_code_high
    uint8_t b6 = 0;               // [6] unknown
    uint8_t b7 = 0;               // [7] unknown
    // Derived receive state
    uint8_t rain = 0;             // ZR rain detect (1 = raining)
    uint16_t unlock_code = 0;     // ZC screen unlock code (16-bit signed)
    uint8_t power_mode = 0;       // CI power mode (1 = ?, 2 = ?)
  };

  // Optional callback for touch/control events from the screen (ZT).
  using NotifyHandler = etl::delegate<void(const uint8_t touch_code)>;

  ScreenDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~ScreenDriver() = default;

  // Send the current screen state (ZA) to the screen MCU.
  void Start();

  // ZA send: 8 bytes [lock][sim][wifi][page_num][err_low][err_high][b6][b7].
  void SetScreenState(uint8_t lock, uint8_t sim, uint8_t wifi, uint8_t page_num,
                      uint8_t err_low, uint8_t err_high, uint8_t b6 = 0, uint8_t b7 = 0);

  // CI send: screen power mode (1-byte payload).
  void SetPowerMode(uint8_t mode);

  // ZT sub-code 6 clears the screen lock (obj+0x135 = 0).
  void ClearScreenLock();

  const ScreenState& GetState() const;

  // Register a callback to be called when the screen sends a touch/control event.
  void RegisterNotifyCallback(const NotifyHandler& handler);

 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_{};
  ScreenState state_;
  NotifyHandler registered_handler_{};

  static inline int16_t ReadI16Le(const uint8_t* data, size_t offset);

  std::vector<uint8_t> EncodeScreenStateCommand();
  std::vector<uint8_t> EncodePowerModeCommand(uint8_t mode);

  // Z? receive handlers (all dispatched by the 'Z' main command)
  void OnZC(const uint8_t* payload, size_t length, uint8_t ack);  // screen unlock code
  void OnZE(const uint8_t* payload, size_t length, uint8_t ack);  // key reset / factory reset
  void OnZR(const uint8_t* payload, size_t length, uint8_t ack);  // rain detect
  void OnZT(const uint8_t* payload, size_t length, uint8_t ack);  // touch/test/control
  void OnCI(const uint8_t* payload, size_t length, uint8_t ack);  // power mode request
};

}  // namespace xbot::driver::screen
