#include "motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace xbot::driver::motor {

namespace {

uint16_t rol16(uint16_t value, uint8_t bits) {
  value &= 0xFFFFu;
  return static_cast<uint16_t>((value << bits) | (value >> (16u - bits)));
}

}  // namespace

MotorDriver::MotorDriver(xbot::driver::mcu::Dispatcher* dispatcher)
    : mcu_driver_(dispatcher), state_{} {
  if (dispatcher) {
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('A'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnMA>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('B'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnMB>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('C'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnMC>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('D'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnMD>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('E'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnME>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('F'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnMF>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('S'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnMS>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('T'),
                               etl::delegate<void(const uint8_t*, size_t)>::create<MotorDriver, &MotorDriver::OnMT>(*this));
  }
}

inline int16_t MotorDriver::ReadI16Le(const uint8_t* data, size_t offset) {
  if (!data) return 0;
  return static_cast<int16_t>(static_cast<uint16_t>(data[offset]) |
                              (static_cast<uint16_t>(data[offset + 1]) << 8));
}

inline uint16_t MotorDriver::ReadU16Le(const uint8_t* data, size_t offset) {
  if (!data) return 0;
  return static_cast<uint16_t>(static_cast<uint16_t>(data[offset]) |
                              (static_cast<uint16_t>(data[offset + 1]) << 8));
}

void MotorDriver::SetStateCallback(const StateCallback& cb) {
  state_callback_ = cb;
}

void MotorDriver::Start() {
  state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED;
  NotifyState();
}

void MotorDriver::RequestStatus() {
  // The MCU link exposes the raw framing layer, but the current dispatcher has no public
  // write API yet. Keep the driver state machine in sync with the protocol by updating
  // the local status on request and let higher layers add the actual transport send when it
  // is exposed by the dispatcher.
  state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  NotifyState();
}

void MotorDriver::SetDuty(float duty) {
  target_duty_ = std::clamp(duty, -1.0f, 1.0f);
  state_.direction = (target_duty_ >= 0.0f) ? 0.0f : 1.0f;
  state_.rpm = std::fabs(target_duty_) * 5000.0f;
  if (std::fabs(target_duty_) < 0.0001f) {
    state_.rpm = 0.0f;
  }
  NotifyState();
}

const MotorDriver::ESCState& MotorDriver::GetState() const {
  return state_;
}

uint16_t MotorDriver::BrandEncode(const std::string& brand, int speed) {
  const uint16_t magnitude = static_cast<uint16_t>(std::abs(speed) & 0xFFFFu);
  if (brand == "dechang") {
    return static_cast<uint16_t>((rol16(magnitude, 1) & 0xFFFEu));
  }
  if (brand == "lianyi") {
    return static_cast<uint16_t>((rol16(magnitude, 2) & 0xFFFCu));
  }
  return magnitude;
}

int16_t MotorDriver::BrandDecode(const std::string& brand, uint16_t raw) {
  int32_t signed_raw = static_cast<int32_t>(raw & 0xFFFFu);
  if (signed_raw & 0x8000) {
    signed_raw -= 0x10000;
  }

  if (brand == "dechang") {
    const uint32_t v = (static_cast<uint32_t>(signed_raw * 2u) | (static_cast<uint32_t>(signed_raw >> 31))) & 0xFFFFFFFFu;
    const uint32_t rolled = ((v << 31) | (v >> 1)) & 0xFFFFFFFFu;
    return static_cast<int16_t>(rolled & 0xFFFFu);
  }

  if (brand == "lianyi") {
    int32_t v = signed_raw + (signed_raw < 0 ? 3 : 0);
    v = v >> 2;
    return static_cast<int16_t>(v & 0xFFFF);
  }

  return static_cast<int16_t>(signed_raw & 0xFFFF);
}

std::vector<uint8_t> MotorDriver::EncodeMAControl(uint8_t type, uint8_t dir_or_brand, int16_t value) {
  std::vector<uint8_t> control(4);
  control[0] = type;
  control[1] = dir_or_brand;
  control[2] = static_cast<uint8_t>(value & 0xFF);
  control[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
  return control;
}

std::vector<uint8_t> MotorDriver::EncodeSpeedCommand(const std::string& brand, int speed) {
  const bool dir = speed >= 0 ? false : true;
  const uint16_t encoded = BrandEncode(brand, std::abs(speed)) & 0xFFFFu;
  std::vector<uint8_t> out(4);
  out[0] = 0x0A;
  out[1] = static_cast<uint8_t>(dir ? 1u : 0u);
  out[2] = static_cast<uint8_t>(encoded & 0xFFu);
  out[3] = static_cast<uint8_t>((encoded >> 8) & 0xFFu);
  return out;
}

std::vector<uint8_t> MotorDriver::EncodeEnableCommand(uint8_t motor_type) {
  return {0x0B, 0x01, motor_type & 0xFFu, 0x00u};
}

std::vector<uint8_t> MotorDriver::EncodeStopCommand() {
  return {0x02, 0x00, 0x00, 0x00};
}

void MotorDriver::NotifyState() {
  if (state_callback_) {
    state_callback_(state_);
  }
}

void MotorDriver::OnMA(const uint8_t* payload, size_t length) {
  if (!payload || length == 0) {
    return;
  }
  for (size_t i = 0; i + 3 < length; i += 4) {
    const uint8_t type = payload[i];
    const uint8_t dir_or_brand = payload[i + 1];
    const int16_t value = ReadI16Le(payload, i + 2);
    ack_ = ack_;
    if (type == 0x0A) {
      state_.direction = ((dir_or_brand & 0x01u) != 0u) ? 1.0f : 0.0f;
      state_.rpm = std::fabs(static_cast<float>(value));
    } else if (type == 0x0B) {
      state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
    }
  }
  NotifyState();
}

void MotorDriver::OnMB(const uint8_t* payload, size_t length) {
  if (!payload || length == 0) {
    return;
  }
  const size_t max_words = std::min<size_t>(length / 2, 5u);
  float sum = 0.0f;
  for (size_t i = 0; i < max_words; ++i) {
    const int16_t value = ReadI16Le(payload, i * 2);
    sum += static_cast<float>(value);
  }
  state_.current_input = sum / static_cast<float>(std::max<size_t>(1u, max_words));
  state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  NotifyState();
}

void MotorDriver::OnMC(const uint8_t* payload, size_t length) {
  if (!payload || length <= 2) {
    return;
  }
  state_.current_input = static_cast<float>(payload[2]);
  state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  NotifyState();
}

void MotorDriver::OnMD(const uint8_t* payload, size_t length) {
  if (!payload || length == 0) {
    return;
  }
  const uint8_t flag = payload[0];
  switch (flag) {
    case 0: state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
    case 1: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
    case 2: state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR; break;
    default: state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
  }
  NotifyState();
}

void MotorDriver::OnME(const uint8_t* payload, size_t length) {
  if (!payload || length == 0) {
    return;
  }
  const uint8_t kind = payload[0];
  switch (kind) {
    case 1: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
    case 2: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
    case 10: state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
    case 4: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
    case 5: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
    case 6: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
    default: state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
  }
  NotifyState();
}

void MotorDriver::OnMF(const uint8_t* payload, size_t length) {
  if (!payload || length == 0) {
    return;
  }
  const uint8_t warning = payload[0];
  if (warning == 4 || warning == 5 || warning == 10) {
    state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
  }
  NotifyState();
}

void MotorDriver::OnMS(const uint8_t* payload, size_t length) {
  if (!payload || length <= 1) {
    return;
  }
  const uint8_t motor_type = payload[0];
  if (motor_type == 10 && length >= 7) {
    const int16_t rpm1 = ReadI16Le(payload, 1);
    const int16_t rpm2 = ReadI16Le(payload, 3);
    const int16_t rpm3 = ReadI16Le(payload, 5);
    state_.rpm = std::fabs(static_cast<float>(rpm1 + rpm2 + rpm3) / 3.0f);
    state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  }
  NotifyState();
}

void MotorDriver::OnMT(const uint8_t* payload, size_t length) {
  if (!payload || length == 0) {
    return;
  }
  const size_t count = std::min<size_t>(4u, length);
  std::string info(reinterpret_cast<const char*>(payload), count);
  if (count < length) {
    info.resize(static_cast<size_t>(count));
  }
  (void)info;
  state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  NotifyState();
}

}  // namespace xbot::driver::motor