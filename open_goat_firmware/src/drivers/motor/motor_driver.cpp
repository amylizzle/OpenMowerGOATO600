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
    // mow motor
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

    // wheel motor
    dispatcher->RegisterHandler(static_cast<uint8_t>('W'), static_cast<uint8_t>('D'),
                            etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnWD>(*this));
    dispatcher->RegisterHandler(static_cast<uint8_t>('W'), static_cast<uint8_t>('R'),
                            etl::delegate<void(const uint8_t*, size_t, uint8_t)>::create<MotorDriver, &MotorDriver::OnWD>(*this));

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
  left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  right_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  mow_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  
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

  auto enable_cmd = EncodeEnableCommand(0x0A); //mow moter enable
  mcu_driver_->SendMessage('M','A',enable_cmd.data(), enable_cmd.size());
  enable_cmd = EncodeEnableCommand(0x0C); //wheel moter enable
  mcu_driver_->SendMessage('W','A',enable_cmd.data(), enable_cmd.size());

  ULOG_INFO("Setting to target rpm: %f, %f, %f", left_state_.target_rpm, right_state_.target_rpm, mow_state_.target_rpm);
  auto mow_cmd = EncodeMowSpeedCommand(mow_state_.brand, mow_state_.target_rpm);
  auto wheel_cmd = EncodeWheelSpeedCommand(left_state_.target_rpm, right_state_.target_rpm);
  
  mcu_driver_->SendMessage('M','A',mow_cmd.data(), mow_cmd.size());
  mcu_driver_->SendMessage('W','A',wheel_cmd.data(), wheel_cmd.size());
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

// mow speed message
std::vector<uint8_t> MotorDriver::EncodeMowSpeedCommand(const ESCState::MotorBrand brand, int speed) {
  const bool dir = speed >= 0 ? false : true;
  const uint16_t encoded = BrandEncode(brand, std::abs(speed)) & 0xFFFFu;
  std::vector<uint8_t> out(4);
  out[0] = 0x0A;
  out[1] = static_cast<uint8_t>(dir ? 1u : 0u);
  out[2] = static_cast<uint8_t>(encoded & 0xFFu);
  out[3] = static_cast<uint8_t>((encoded >> 8) & 0xFFu);
  return out;
}

// wheel speed message (mm/s maybe?)
std::vector<uint8_t> MotorDriver::EncodeWheelSpeedCommand(int left, int right) {
    std::vector<uint8_t> p(19, 0x00);
    auto put = [&p](int speed, size_t signOff, size_t valOff) {
        int mag = std::abs(speed);
        uint16_t m = (uint16_t)(mag & 0xFFFF);
        p[signOff] = (speed < 0) ? 1 : 0;
        p[valOff]     = (uint8_t)(m & 0xFF);
        p[valOff + 1] = (uint8_t)(m >> 8);
    };
    put(left, 1, 2);
    put(right, 10, 11);
    return p;
}

std::vector<uint8_t> MotorDriver::EncodeEnableCommand(uint8_t motor_type) {
  return {static_cast<uint8_t>(0x0B), static_cast<uint8_t>(0x01),
          static_cast<uint8_t>(motor_type & 0xFFu), static_cast<uint8_t>(0x00)};
}

// motor type not specified for some reason?
std::vector<uint8_t> MotorDriver::EncodeStopCommand() {
  return {static_cast<uint8_t>(0x02), static_cast<uint8_t>(0x00),
          static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00)};
}

// all motor current report (mA I'm assuming)
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
        case 0: left_state_.current_input = static_cast<float>(value)/1000.0f; break; 
        case 1: right_state_.current_input = static_cast<float>(value)/1000.0f; break;
        case 2: mow_state_.current_input = static_cast<float>(value)/1000.0f; break;
        default: break; //5 channels, one packing gap, channels 0,1,2 are current (maybe) 
    }
  }
  
}

// MOW motor current response from request? Weird. MB messages stream current though.
void MotorDriver::OnMC(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length <= 2) {
    return;
  }
  ULOG_WARNING("[MOTOR] MC: ack %u val: %u", ack, static_cast<unsigned>(payload[2]));
  mow_state_.current_input = static_cast<float>(payload[2]);   
}

// MOW motor distance report? 
void MotorDriver::OnMD(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t flag = payload[0];
  ULOG_WARNING("[MOTOR] MD: ack %u flag: %u", ack, static_cast<unsigned>(flag));
//   switch (flag) {
//     case 0: left_state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
//     case 1: left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
//     case 2: left_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR; break;
//     default: left_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR; break;
//   } 
}

// MOW motor error? TODO
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

// MOW motor warning states? TODO
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

// MOW motor status 
void MotorDriver::OnMS(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length <= 1) {
    return;
  }
  const uint8_t motor_type = payload[0];
  const int16_t rpm1 = ReadI16Le(payload, 1);
  const int16_t rpm2 = ReadI16Le(payload, 3);
  ULOG_WARNING("[MOTOR] MS: ack %u type %u RPM - %d %d", ack, motor_type, rpm1, rpm2);
  mow_state_.rpm = static_cast<float>(rpm1);
  mow_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
}

// MOW motor text logging from the MCU? weird
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
}


// Wheel motor distance report? 
void MotorDriver::OnWD(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t flag = payload[0];
  ULOG_WARNING("[MOTOR] WD: ack %u flag: %u", ack, static_cast<unsigned>(flag));
//   switch (flag) {
//     case 0: left_state_.status = ESCState::ESCStatus::ESC_STATUS_DISCONNECTED; break;
//     case 1: left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK; break;
//     case 2: left_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR; break;
//     default: left_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR; break;
//   } 
}

// Wheel motor status 
void MotorDriver::OnWR(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length <= 1) {
    return;
  }
  const uint8_t motor_type = payload[0];
  const int16_t rpm1 = ReadI16Le(payload, 9);
  const int16_t rpm2 = ReadI16Le(payload, 13);
  ULOG_WARNING("[MOTOR] WR: ack %u type %u RPM - %d %d", ack, motor_type, rpm1, rpm2);
  mow_state_.rpm = static_cast<float>(rpm1);
  mow_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
}

}  // namespace xbot::driver::motor