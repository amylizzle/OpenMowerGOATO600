#include "screen_driver.hpp"
#include <ulog.h>
#include <algorithm>

namespace xbot::driver::screen {

ScreenDriver::ScreenDriver(xbot::driver::mcu::Dispatcher* dispatcher)
    : mcu_driver_(dispatcher) {
    // ScreenInfo receive ids: ZC, ZE, ZR, ZT (all main cmd 'Z')
    dispatcher->RegisterHandler(static_cast<uint8_t>('Z'), static_cast<uint8_t>('C'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<ScreenDriver, &ScreenDriver::OnZC>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('Z'), static_cast<uint8_t>('E'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<ScreenDriver, &ScreenDriver::OnZE>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('Z'), static_cast<uint8_t>('R'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<ScreenDriver, &ScreenDriver::OnZR>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('Z'), static_cast<uint8_t>('T'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<ScreenDriver, &ScreenDriver::OnZT>(*this));

    // CI power mode request (main cmd 'C')
    dispatcher->RegisterHandler(static_cast<uint8_t>('C'), static_cast<uint8_t>('I'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<ScreenDriver, &ScreenDriver::OnCI>(*this));
}

inline int16_t ScreenDriver::ReadI16Le(const uint8_t* data, size_t offset) {
  if (!data) return 0;
  return static_cast<int16_t>(static_cast<uint16_t>(data[offset]) |
                              (static_cast<uint16_t>(data[offset + 1]) << 8));
}

// ZA send: 8 bytes [lock][internet][wifi][page_num][err_low][err_high][pincode_confirm][pincode_first].
std::vector<uint8_t> ScreenDriver::EncodeScreenStateCommand() {
  std::vector<uint8_t> out(8);
  out[0] = state_.screen_lock;
  out[1] = state_.internet;
  out[2] = state_.wifi;
  out[3] = state_.page_num;
  out[4] = state_.err_code_low;
  out[5] = state_.err_code_high;
  out[6] = state_.pincode_confirm;
  out[7] = state_.pincode_first;
  return out;
}

// CI power mode: 1-byte payload.
std::vector<uint8_t> ScreenDriver::EncodePowerModeCommand(ScreenPowerState mode) {
  return {static_cast<uint8_t>(mode & 0xFFu)};
}

void ScreenDriver::Start() {
  SetPowerMode(ScreenPowerState::POWER_NORMAL);
  SetScreenState(
    ScreenIconState::ICON_OFF,
    ScreenIconState::ICON_OFF,
    ScreenIconState::ICON_OFF,
    ScreenPage::ON,
    0,
    0,
    0,
    0);
}

void ScreenDriver::SetScreenState(ScreenIconState lock, ScreenIconState internet, ScreenIconState wifi, ScreenPage page_num,
                                  uint8_t err_low, uint8_t err_high, uint8_t pincode_confirm, uint8_t pincode_first) {
  state_.screen_lock = lock;
  state_.internet = internet;
  state_.wifi = wifi;
  state_.page_num = page_num;
  state_.err_code_low = err_low;
  state_.err_code_high = err_high;
  state_.pincode_confirm = pincode_confirm;
  state_.pincode_first = pincode_first;

  auto cmd = EncodeScreenStateCommand();
  mcu_driver_->SendMessage('Z', 'A', cmd.data(), cmd.size());
}

void ScreenDriver::SetPowerMode(ScreenPowerState mode) {
  if (state_.power_mode == mode) {
    return;
  }
  state_.power_mode = mode;
  auto cmd = EncodePowerModeCommand(mode);
  mcu_driver_->SendMessage('C', 'I', cmd.data(), cmd.size());
}

void ScreenDriver::SetLockIcon(ScreenIconState value){
  if (state_.screen_lock == value) {
    return;
  }
  state_.screen_lock = value;
  auto cmd = EncodeScreenStateCommand();
  mcu_driver_->SendMessage('Z', 'A', cmd.data(), cmd.size());  
}

void ScreenDriver::SetInternetIcon(ScreenIconState value){
  if (state_.internet == value) {
    return;
  }
  state_.internet = value;
  auto cmd = EncodeScreenStateCommand();
  mcu_driver_->SendMessage('Z', 'A', cmd.data(), cmd.size());  
}

void ScreenDriver::SetWifiIcon(ScreenIconState value){
  if (state_.wifi == value) {
    return;
  }
  state_.wifi = value;
  auto cmd = EncodeScreenStateCommand();
  mcu_driver_->SendMessage('Z', 'A', cmd.data(), cmd.size());  
}

void ScreenDriver::SetScreenPage(ScreenPage value){
  if (state_.page_num == value) {
    return;
  }
  state_.page_num = value;
  auto cmd = EncodeScreenStateCommand();
  mcu_driver_->SendMessage('Z', 'A', cmd.data(), cmd.size());  
}

void ScreenDriver::SetErrorCode(uint16_t num, bool show_error_page){
  if (state_.err_code_low == (num & 0x00FF) && state_.err_code_high == ((num & 0xFF00) >> 8)) {
    return;
  }
  state_.err_code_high = (num & 0xFF00) >> 8;
  state_.err_code_low = (num & 0x00FF);
  auto cmd = EncodeScreenStateCommand();
  mcu_driver_->SendMessage('Z', 'A', cmd.data(), cmd.size()); 
  if(show_error_page)
    SetScreenPage(ScreenPage::ERROR); 
}


void ScreenDriver::RegisterNotifyCallback(const NotifyHandler& handler) {
  registered_handler_ = handler;
}

// ZT touch/test/control: byte at data+0 is the sub-code.
//   0xd key-shutdown, 9 wifi_mark, 4 factory test, 6 clear stop, 0xe roll-motor
void ScreenDriver::OnZT(const uint8_t* payload, size_t length, uint8_t ack) {
  (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t code = payload[0];
  ULOG_WARNING("[SCREEN] ZT: ack %u code: %u", ack, static_cast<unsigned>(code));

  if (registered_handler_) {
    registered_handler_(code);
  }
}

// ZC screen lock/unlock code: 16-bit signed code from data[0..1].
void ScreenDriver::OnZC(const uint8_t* payload, size_t length, uint8_t ack) {
  (void) ack;
  if (!payload || length < 2) {
    return;
  }
  const uint16_t code = ReadI16Le(payload, 0);
  state_.unlock_code = code;
  ULOG_DEBUG("[SCREEN] ZC: ack %u unlock code: %d", ack, static_cast<int16_t>(code));
}

// ZR rain detect: data[0]==1 -> state = data[1] (1 rain / 0 none).
void ScreenDriver::OnZR(const uint8_t* payload, size_t length, uint8_t ack) {
  (void) ack;
  if (!payload || length < 2) {
    return;
  }
  if (payload[0] == 1) {
    const uint8_t rain = (payload[1] == 1) ? 1u : 0u;
    if (state_.rain != rain) {
      state_.rain = rain;
      ULOG_WARNING("[SCREEN] ZR: ack %u mcu rain detect: %u", ack, static_cast<unsigned>(rain));
    }
  }
}

// ZE screen key reset / factory reset event (no data fields).
void ScreenDriver::OnZE(const uint8_t* payload, size_t length, uint8_t ack) {
  (void) payload;
  (void) length;
  ULOG_WARNING("[SCREEN] ZE: ack %u screen key reset event", ack);
}

// CI power mode: received data[0]==3 -> send 'I' + byte 2; ==2 -> send 'I' + byte 1; else skip.
void ScreenDriver::OnCI(const uint8_t* payload, size_t length, uint8_t ack) {
  (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t mode = payload[0];
  ULOG_WARNING("[SCREEN] CI: ack %u screen power mode: %u", ack, static_cast<unsigned>(mode));
}

}  // namespace xbot::driver::screen
