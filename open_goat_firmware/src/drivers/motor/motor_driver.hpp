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
    float direction = 0.0f; // 0..1
    ESCStatus status = ESCStatus::ESC_STATUS_DISCONNECTED;
  };

  using StateCallback = etl::delegate<void(const ESCState&)>;

  MotorDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~MotorDriver() = default;

  void SetStateCallback(const StateCallback& cb);
  void Start();
  void RequestStatus();
  void SetDuty(float duty); // duty in [-1,1]
  const ESCState& GetState() const;

  // Python protocol helpers mirrored from the MCU node implementation.
  static uint16_t BrandEncode(const std::string& brand, int speed);
  static int16_t BrandDecode(const std::string& brand, uint16_t raw);
  static std::vector<uint8_t> EncodeMAControl(uint8_t type, uint8_t dir_or_brand, int16_t value);
  static std::vector<uint8_t> EncodeSpeedCommand(const std::string& brand, int speed);
  static std::vector<uint8_t> EncodeEnableCommand(uint8_t motor_type);
  static std::vector<uint8_t> EncodeStopCommand();

 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_{};
  ESCState state_{};
  StateCallback state_callback_{};
  float target_duty_ = 0.0f;
  uint8_t ack_ = 0;

  static inline int16_t ReadI16Le(const uint8_t* data, size_t offset);
  static inline uint16_t ReadU16Le(const uint8_t* data, size_t offset);

  void NotifyState();
  void OnMA(const uint8_t* payload, size_t length);
  void OnMB(const uint8_t* payload, size_t length);
  void OnMC(const uint8_t* payload, size_t length);
  void OnMD(const uint8_t* payload, size_t length);
  void OnME(const uint8_t* payload, size_t length);
  void OnMF(const uint8_t* payload, size_t length);
  void OnMS(const uint8_t* payload, size_t length);
  void OnMT(const uint8_t* payload, size_t length);
};

} // namespace xbot::driver::motor
