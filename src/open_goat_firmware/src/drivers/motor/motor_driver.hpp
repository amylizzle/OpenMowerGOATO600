// Minimal Motor (ESC) driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>

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
    float direction = 0.0f; // 0..1
    ESCStatus status = ESCStatus::ESC_STATUS_DISCONNECTED;
  };

  using StateCallback = etl::delegate<void(const ESCState&)>;

  virtual ~MotorDriver() = default;

  virtual void SetStateCallback(const StateCallback& cb) = 0;
  virtual void Start() = 0;
  virtual void RequestStatus() = 0;
  virtual void SetDuty(float duty) = 0; // duty in [-1,1]
};

} // namespace xbot::driver::motor
