// Minimal IMU driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>


namespace xbot::driver::imu {

class ImuDriver {
 private:
  xbot::driver::mcu::Dispatcher* mcu_driver_; 
  mutable struct Data {
    bool valid = false;
    int16_t gyro[4] = {0, 0, 0, 0};
    int16_t accel[3] = {0, 0, 0};
    int16_t mag[3] = {0, 0, 0};
    uint32_t ts = 0;
    // Gyro bias (GF) - six unsigned 16-bit values
    uint16_t bias[6] = {0, 0, 0, 0, 0, 0};
    // Geomag (GH) validity: two u16 and one u8
    uint16_t geomag_u16[2] = {0, 0};
    uint8_t geomag_u8 = 0;
    // Geomag (GI): three u16 values
    uint16_t geomag3[3] = {0, 0, 0};
    // GS state/value
    uint8_t state = 0;
    uint8_t state_value = 0;
    // OD fields
    uint8_t ultrasonic = 0;
    uint8_t gyro_type = 0;
  } data_{};

 public:
  ImuDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~ImuDriver() = default;

  // Initialize hardware / bus
  void Start();

  // Read latest axes data. Expect length==9 (3 accel, 3 gyro, 3 reserved) or similar.
  void ReadAxes(double* axes, size_t length);
  Data GetData();

 private:
  const double accel_scale_factor = 0.01; //cm/s^2 -> m/s^2
  const double gyro_scale_factor = 1.0; //0.01745329; // degrees -> radians
  void OnGD(const uint8_t *payload, size_t length);
  void OnGF(const uint8_t *payload, size_t length);
  void OnGH(const uint8_t *payload, size_t length);
  void OnGI(const uint8_t *payload, size_t length);
  void OnGS(const uint8_t *payload, size_t length);
  void OnOD(const uint8_t *payload, size_t length);
};

} // namespace xbot::driver::imu
