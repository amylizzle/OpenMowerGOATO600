#include "motor_driver.hpp"
#include <ulog.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace xbot::driver::motor {

namespace {

uint16_t rol16(uint16_t value, uint8_t bits) {
  value &= 0xFFFFu;
  return static_cast<uint16_t>((value << bits) | (value >> (16u - bits)));
}

}  // namespace

MotorDriver::MotorDriver(xbot::driver::mcu::Dispatcher* dispatcher)
    : mcu_driver_(dispatcher) {
  if (dispatcher) {
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('A'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnMA>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('B'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnMB>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('C'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnMC>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('D'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnMD>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('E'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnME>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('F'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnMF>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('S'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnMS>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('M'), static_cast<uint8_t>('T'),
                               etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnMT>(*this));
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

void MotorDriver::Start() {
  left_state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED;
  right_state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED;
  mow_state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED;
  
}


void MotorDriver::SetDuty(std::optional<float> left, std::optional<float> right, std::optional<float> mow) {
  if (left.has_value()){
    left_state_.target_duty = std::clamp(left.value(), -1.0f, 1.0f);
    left_state_.direction = (left_state_.target_duty >= 0.0f) ? 0.0f : 1.0f;
    left_state_.target_rpm = std::fabs(left_state_.target_duty) * left_state_.max_rpm;
  }
  if (right.has_value()){
    right_state_.target_duty = std::clamp(right.value(), -1.0f, 1.0f);
    right_state_.direction = (right_state_.target_duty >= 0.0f) ? 0.0f : 1.0f;
    right_state_.target_rpm = std::fabs(right_state_.target_duty) * right_state_.max_rpm;
  }
  if (mow.has_value()){
    mow_state_.target_duty = std::clamp(mow.value(), -1.0f, 1.0f);
    mow_state_.direction = (mow_state_.target_duty >= 0.0f) ? 0.0f : 1.0f;
    mow_state_.target_rpm = std::fabs(mow_state_.target_duty) * mow_state_.max_rpm;
  }
  if (std::fabs(left_state_.target_duty) < 0.0001f) {
    left_state_.target_rpm = 0.0f;
  }
  if (std::fabs(right_state_.target_duty) < 0.0001f) {
    right_state_.target_rpm = 0.0f;
  }
  if (std::fabs(mow_state_.target_duty) < 0.0001f) {
    mow_state_.target_rpm = 0.0f;
  }


  ULOG_INFO("Setting to target rpm: %u, %u, %u",left_state_.target_rpm, right_state_.target_rpm ,mow_state_.target_rpm);
  std::vector<uint8_t> ctl_message;
  auto left_cmd = EncodeSpeedCommand(left_state_.brand, left_state_.target_rpm);
  auto right_cmd = EncodeSpeedCommand(right_state_.brand, right_state_.target_rpm);
  auto mow_cmd = EncodeSpeedCommand(mow_state_.brand, mow_state_.target_rpm);

  ctl_message.reserve(left_cmd.size() + right_cmd.size() + mow_cmd.size());
  ctl_message.insert(ctl_message.end(), left_cmd.begin(), left_cmd.end());
  ctl_message.insert(ctl_message.end(), right_cmd.begin(), right_cmd.end());
  ctl_message.insert(ctl_message.end(), mow_cmd.begin(), mow_cmd.end());

  mcu_driver_->SendMessage('M','A',ctl_message.data(), ctl_message.size());
}

const MotorDriver::ESCState& MotorDriver::GetLeftState() const {
  return left_state_;
}
const MotorDriver::ESCState& MotorDriver::GetRightState() const {
  return right_state_;
}
const MotorDriver::ESCState& MotorDriver::GetMowState() const {
  return mow_state_;
}

uint16_t MotorDriver::BrandEncode(const ESCState::MotorBrand brand, int speed) {
  const uint16_t magnitude = static_cast<uint16_t>(std::abs(speed) & 0xFFFFu);
  if (brand == ESCState::MotorBrand::BRAND_DECHANG) {
    return static_cast<uint16_t>((rol16(magnitude, 1) & 0xFFFEu));
  }
  if (brand == ESCState::MotorBrand::BRAND_LIANYI) {
    return static_cast<uint16_t>((rol16(magnitude, 2) & 0xFFFCu));
  }
  return magnitude;
}

int16_t MotorDriver::BrandDecode(const ESCState::MotorBrand brand, uint16_t raw) {
  int32_t signed_raw = static_cast<int32_t>(raw & 0xFFFFu);
  if (signed_raw & 0x8000) {
    signed_raw -= 0x10000;
  }

  if (brand == ESCState::MotorBrand::BRAND_DECHANG) {
    const uint32_t v = (static_cast<uint32_t>(signed_raw * 2u) | (static_cast<uint32_t>(signed_raw >> 31))) & 0xFFFFFFFFu;
    const uint32_t rolled = ((v << 31) | (v >> 1)) & 0xFFFFFFFFu;
    return static_cast<int16_t>(rolled & 0xFFFFu);
  }

  if (brand == ESCState::MotorBrand::BRAND_LIANYI) {
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

std::vector<uint8_t> MotorDriver::EncodeSpeedCommand(const ESCState::MotorBrand brand, int speed) {
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
  return {static_cast<uint8_t>(0x0B), static_cast<uint8_t>(0x01),
          static_cast<uint8_t>(motor_type & 0xFFu), static_cast<uint8_t>(0x00)};
}

std::vector<uint8_t> MotorDriver::EncodeStopCommand() {
  return {static_cast<uint8_t>(0x02), static_cast<uint8_t>(0x00),
          static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00)};
}

void MotorDriver::OnMA(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  ULOG_INFO("[MOTOR] MA Signal ack %u len %zu", ack, length);
  for (size_t i = 0; i + 3 < length; i += 4) {
    const uint8_t type = payload[i];
    const uint8_t dir_or_brand = payload[i + 1];
    const int16_t value = ReadI16Le(payload, i + 2);
    ack_ = ack_;
    switch(i){
        case 0:
            if (type == 0x0A) {
                left_state_.direction = ((dir_or_brand & 0x01u) != 0u) ? 1.0f : 0.0f;
                left_state_.rpm = std::fabs(static_cast<float>(value));
            } else if (type == 0x0B) {
                left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
            }
            break;
        case 4:
            if (type == 0x0A) {
                left_state_.direction = ((dir_or_brand & 0x01u) != 0u) ? 1.0f : 0.0f;
                left_state_.rpm = std::fabs(static_cast<float>(value));
            } else if (type == 0x0B) {
                left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
            }
            break;
        case 8:
            if (type == 0x0A) {
                left_state_.direction = ((dir_or_brand & 0x01u) != 0u) ? 1.0f : 0.0f;
                left_state_.rpm = std::fabs(static_cast<float>(value));
            } else if (type == 0x0B) {
                left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
            }
            break;  
        default:
            ULOG_INFO("[MOTOR] MA Signal i:%i dir: %u rpm: %u",i, dir_or_brand, value); break; 
        }                 
  }
  
}

void MotorDriver::OnMB(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  ULOG_INFO("[MOTOR] MB Signal ack %u len %zu", ack, length);
  const size_t max_words = std::min<size_t>(length / 2, 6u);
  for (size_t i = 0; i < max_words; ++i) {
    const int16_t value = ReadI16Le(payload, i * 2);
    ULOG_WARNING("[MOTOR] MB i:%zu val: %d", i, value);
    switch(i) {
        case 0: left_state_.current_input = static_cast<float>(value); break;
        case 1: right_state_.current_input = static_cast<float>(value); break;
        case 2: mow_state_.current_input = static_cast<float>(value); break;
        default: break;
    }
  }
  
}

void MotorDriver::OnMC(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length <= 2) {
    return;
  }
  ULOG_WARNING("[MOTOR] MC: ack %u val: %u", ack, static_cast<unsigned>(payload[2]));
//   state_.current_input = static_cast<float>(payload[2]);
//   state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  
}

void MotorDriver::OnMD(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t flag = payload[0];
  ULOG_WARNING("[MOTOR] MD: ack %u flag: %u", ack, static_cast<unsigned>(flag));
  switch (flag) {
    case 0: left_state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
    case 1: left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
    case 2: left_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR; break;
    default: left_state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
  }
  
}

void MotorDriver::OnME(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t kind = payload[0];
  ULOG_WARNING("[MOTOR] ME: ack %u kind %u", ack, static_cast<unsigned>(kind));
//   switch (kind) {
//     case 1: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
//     case 2: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
//     case 10: state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
//     case 4: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
//     case 5: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
//     case 6: state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
//     default: state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
//   }
  
}

void MotorDriver::OnMF(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t warning = payload[0];
  ULOG_WARNING("[MOTOR] MF: ack %u warn: %u", ack, static_cast<unsigned>(warning));
//   if (warning == 4 || warning == 5 || warning == 10) {
//     state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
//   }
  
}

void MotorDriver::OnMS(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length <= 1) {
    return;
  }
  const uint8_t motor_type = payload[0];
  ULOG_WARNING("[MOTOR] MS: ack %u type %u", ack, static_cast<unsigned>(motor_type));
  if (motor_type == 10 && length >= 7) {
    const int16_t rpm1 = ReadI16Le(payload, 1);
    const int16_t rpm2 = ReadI16Le(payload, 3);
    const int16_t rpm3 = ReadI16Le(payload, 5);
    ULOG_WARNING("[MOTOR] RPM - %d %d %d", rpm1, rpm2, rpm3);
    left_state_.rpm = static_cast<float>(rpm1);
    right_state_.rpm = static_cast<float>(rpm2);
    mow_state_.rpm = static_cast<float>(rpm3);
    left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
    right_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
    mow_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
    
  }
  
}

void MotorDriver::OnMT(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const size_t count = std::min<size_t>(4u, length);
  std::string info(reinterpret_cast<const char*>(payload), count);
  if (count < length) {
    info.resize(static_cast<size_t>(count));
  }
  ULOG_WARNING("[MOTOR] MT: ack %u info: %s", ack, info.c_str());
//   state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  
}

}  // namespace xbot::driver::motor