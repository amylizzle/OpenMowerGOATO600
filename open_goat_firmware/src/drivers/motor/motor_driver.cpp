#include "motor_driver.hpp"
#include "misc_utils.h"
#include <ulog.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace xbot::driver::motor {

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

void MotorDriver::Start() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  left_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  right_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  mow_state_.status = ESCState::ESCStatus::ESC_STATUS_OK;
  mow_state_.max_rpm = 3000.0f; //seems to be the set point in the MCU firmware

  auto enable_cmd = EncodeMowSpeedCommand(mow_state_.target_rpm);
  mcu_driver_->SendMessage('M','A',enable_cmd.data(), enable_cmd.size());
  enable_cmd = EncodeWheelSpeedCommand(left_state_.target_rpm, right_state_.target_rpm);
  mcu_driver_->SendMessage('W','A',enable_cmd.data(), enable_cmd.size());
  // Both the diff drive and mower services share one driver instance and each
  // call Start(), so only spawn the message loop on the first call.
  if (processing_thread_ == nullptr) {
    processing_thread_ = createThread(ThreadEntry, this);
  }
}

void MotorDriver::ThreadEntry(void* arg) {
	auto inst = static_cast<MotorDriver*>(arg);
	if (inst) MotorMessageLoop(inst);
}

void MotorDriver::MotorMessageLoop(MotorDriver* instance) {
  while(true){
    float mow_rpm, left_rpm, right_rpm;
    {
      std::lock_guard<std::mutex> lock(instance->state_mutex_);
      mow_rpm = instance->mow_state_.target_rpm;
      left_rpm = instance->left_state_.target_rpm;
      right_rpm = instance->right_state_.target_rpm;
    }
    auto mow_cmd = instance->EncodeMowSpeedCommand(mow_rpm);
    auto wheel_cmd = instance->EncodeWheelSpeedCommand(left_rpm, right_rpm);

    instance->mcu_driver_->SendMessage('M','A',mow_cmd.data(), mow_cmd.size());
    instance->mcu_driver_->SendMessage('W','A',wheel_cmd.data(), wheel_cmd.size());
    std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(0.02)) //50Hz
    );
  }
}

void MotorDriver::SetDuty(std::optional<float> left, std::optional<float> right, std::optional<float> mow) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (left.has_value()){
    left_state_.target_duty = std::clamp(left.value(), -1.0f, 1.0f);
    left_state_.target_rpm = left_state_.target_duty * left_state_.max_rpm;
  }
  if (right.has_value()){
    right_state_.target_duty = std::clamp(right.value(), -1.0f, 1.0f);
    right_state_.target_rpm = right_state_.target_duty * right_state_.max_rpm;
  }
  if (mow.has_value()){
    mow_state_.target_duty = std::clamp(mow.value(), -1.0f, 1.0f);
    mow_state_.target_rpm = mow_state_.target_duty * mow_state_.max_rpm;
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

  ULOG_DEBUG("Setting to target rpm: %f, %f, %f", left_state_.target_rpm, right_state_.target_rpm, mow_state_.target_rpm);
  
}

const MotorDriver::ESCState MotorDriver::GetLeftState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return left_state_;
}
const MotorDriver::ESCState MotorDriver::GetRightState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return right_state_;
}
const MotorDriver::ESCState MotorDriver::GetMowState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return mow_state_;
}

// mow speed message
std::vector<uint8_t> MotorDriver::EncodeMowSpeedCommand(int speed) {
  std::vector<uint8_t> out(4);
  //0x0A is mow motor, 0x0B is lift motor, 0x0C is roll motor
  out[0] = 0x0A;
  out[1] = 0x06; //constant
  out[2] = static_cast<uint8_t>((speed & 0xFF));
  out[3] = static_cast<uint8_t>((speed >> 8));
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

// all motor current report (mA)
void MotorDriver::OnMB(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  ULOG_DEBUG("[MOTOR] MB Signal ack %u len %zu", ack, length);
  std::lock_guard<std::mutex> lock(state_mutex_);
  const size_t max_words = std::min<size_t>(length / 2, 6u);
  for (size_t i = 0; i < max_words; ++i) {
    const int16_t value = ReadI16Le(payload, i * 2);
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
  std::lock_guard<std::mutex> lock(state_mutex_);
  mow_state_.current_input = static_cast<float>(payload[2]);   
}

// MOW motor distance report? 
void MotorDriver::OnMD(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t flag = payload[0];
  ULOG_WARNING("[MOTOR] MD: ack %u flag: %u len: %u", ack, static_cast<unsigned>(flag), length);
}

// MOW motor error? TODO
void MotorDriver::OnME(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t kind = payload[0];
  ULOG_ERROR("[MOTOR] ME: ack %u len %u kind %u", ack, length, static_cast<unsigned>(kind));
  // let's just set an error state if this happens, since we don't know what it means
  std::lock_guard<std::mutex> lock(state_mutex_);
  mow_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
  left_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
  right_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
  
}

// MOW motor warning states? TODO
void MotorDriver::OnMF(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length == 0) {
    return;
  }
  const uint8_t warning = payload[0];
  ULOG_ERROR("[MOTOR] MF: ack %u len %u warn: %u", ack, length, static_cast<unsigned>(warning));
  // let's just set an error state if this happens, since we don't know what it means
  std::lock_guard<std::mutex> lock(state_mutex_);
  mow_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
  left_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
  right_state_.status = ESCState::ESCStatus::ESC_STATUS_ERROR;
}

// MOW motor status - has 5 RPM values, but only the first is used for the MOW motor. The rest are for other motors that aren't used.
void MotorDriver::OnMS(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length <= 1) {
    return;
  }
  const uint8_t motor_type = payload[0];
  const int16_t rpm1 = ReadI16Le(payload, 1);
  const int16_t rpm2 = ReadI16Le(payload, 3);
  ULOG_DEBUG("[MOTOR] MS: ack %u type %u RPM - %d %d", ack, motor_type, rpm1, rpm2);
  std::lock_guard<std::mutex> lock(state_mutex_);
  mow_state_.rpm = static_cast<float>(rpm1);
}

// Motor (or maybe ESC) temperature
void MotorDriver::OnMT(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length < 3) {
    return;
  }

  left_state_.temperature_pcb = static_cast<float>(payload[0]);
  right_state_.temperature_pcb = static_cast<float>(payload[1]);
  mow_state_.temperature_pcb = static_cast<float>(payload[2]);
}


// Wheel motor distance report 
void MotorDriver::OnWD(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length < 13) {
    return;
  }
  int32_t left = ReadI32Le(payload, 1);
  int32_t right = ReadI32Le(payload, 5);
  uint32_t timestamp = ReadU32Le(payload, 9);
  ULOG_DEBUG("[MOTOR] WD: ack %u left %d right %d timestamp %u", ack, left, right, timestamp);
  std::lock_guard<std::mutex> lock(state_mutex_);

  // First frame we have ever seen: nothing to compare against, just take it.
  if (!has_wd_) {
    AcceptWdReset(left, right, timestamp);
    return;
  }

  // A timestamp discontinuity means the MCU restarted its counters (docking
  // does this) or we lost a long run of frames. Either way the distance
  // travelled in between is unknown, so confirm over a few frames and then
  // re-baseline rather than differencing across the gap.
  if (abs(static_cast<int32_t>(wd_last_timestamp - timestamp)) > 1000) {
    if (++bad_wd_count_ > 3) {
      ULOG_WARNING("[MOTOR] WD: multiple bad timestamps detected, re-baselining at %u", timestamp);
      AcceptWdReset(left, right, timestamp);
    }
    return;
  }

  // Sometimes invalid values are sent; ignore them. 200 ticks in 20ms is well
  // over 2m/s, which is faster than the mower can go. Checked per wheel, but
  // the frame is dropped as a unit so the two counters stay on the same
  // timestamp. If the implausible values persist then the counters really did
  // move, so re-baseline instead of freezing the odometry forever.
  if (abs(left_state_.tacho - left) > 200 || abs(right_state_.tacho - right) > 200) {
    if (++bad_wd_count_ > 3) {
      ULOG_WARNING("[MOTOR] WD: persistent tick discontinuity, re-baselining to %d / %d", left, right);
      AcceptWdReset(left, right, timestamp);
    }
    return;
  }

  bad_wd_count_ = 0;
  wd_last_timestamp = timestamp;
  left_state_.tacho = left;
  right_state_.tacho = right;
}

void MotorDriver::AcceptWdReset(int32_t left, int32_t right, uint32_t timestamp) {
  bad_wd_count_ = 0;
  has_wd_ = true;
  wd_last_timestamp = timestamp;
  left_state_.tacho = left;
  right_state_.tacho = right;
  // Tell consumers that a delta spanning this frame is meaningless.
  ++tacho_epoch_;
  left_state_.tacho_epoch = tacho_epoch_;
  right_state_.tacho_epoch = tacho_epoch_;
}


// Wheel motor status - never sent?
void MotorDriver::OnWR(const uint8_t* payload, size_t length, uint8_t ack) {
    (void) ack;
  if (!payload || length <= 1) {
    return;
  }
  const uint8_t motor_type = payload[0];
  const int16_t rpm1 = ReadI16Le(payload, 9);
  const int16_t rpm2 = ReadI16Le(payload, 13);
  ULOG_WARNING("[MOTOR] WR: ack %u type %u RPM - %d %d", ack, motor_type, rpm1, rpm2);
  std::lock_guard<std::mutex> lock(state_mutex_);
  left_state_.rpm = static_cast<float>(rpm1);
  right_state_.rpm = static_cast<float>(rpm2);
}

}  // namespace xbot::driver::motor