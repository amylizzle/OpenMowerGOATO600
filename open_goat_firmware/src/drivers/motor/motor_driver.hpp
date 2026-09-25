// Minimal Motor (ESC) driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>

namespace xbot::driver::motor {

class MotorDriver {
 public:
  struct ESCState {
    enum class ESCStatus : uint8_t { ESC_STATUS_DISCONNECTED = 0, ESC_STATUS_OK = 1, ESC_STATUS_ERROR = 2 };
    int32_t tacho = 0;
    float temperature_pcb = 0.0f;
    float temperature_motor = 0.0f;
    float current_input = 0.0f;
    float rpm = 0.0f;
    float target_duty = 0.0f;
    float target_rpm = 0.0f;
    float max_rpm = 2000.0f;
    ESCStatus status = ESCStatus::ESC_STATUS_OK;
  };

  MotorDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~MotorDriver() = default;

  void Start();
  void SetDuty(std::optional<float> left, std::optional<float> right, std::optional<float> mow); // duty in [-1,1], std::nullopt for don't change
  const ESCState GetLeftState() const;
  const ESCState GetRightState() const;
  const ESCState GetMowState() const;

 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_{};
  mutable std::mutex state_mutex_;
  ESCState left_state_;
  ESCState right_state_;
  ESCState mow_state_;
  uint8_t bad_wd_count_ = 0;
  uint32_t wd_last_timestamp = 0;
  thread_t *processing_thread_ = nullptr;
  static void MotorMessageLoop(MotorDriver* instance);
  static void ThreadEntry(void* arg);

  std::vector<uint8_t> EncodeMowSpeedCommand(int speed);
  std::vector<uint8_t> EncodeWheelSpeedCommand(int left, int right);

  void OnMB(const uint8_t* payload, size_t length, uint8_t ack);
  void OnMC(const uint8_t* payload, size_t length, uint8_t ack);
  void OnMD(const uint8_t* payload, size_t length, uint8_t ack);
  void OnME(const uint8_t* payload, size_t length, uint8_t ack);
  void OnMF(const uint8_t* payload, size_t length, uint8_t ack);
  void OnMS(const uint8_t* payload, size_t length, uint8_t ack);
  void OnMT(const uint8_t* payload, size_t length, uint8_t ack);
  void OnWD(const uint8_t* payload, size_t length, uint8_t ack);
  void OnWR(const uint8_t* payload, size_t length, uint8_t ack);
};

} // namespace xbot::driver::motor
