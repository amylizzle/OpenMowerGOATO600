// Minimal IMU driver interface (MCU-backed stub)
#pragma once

#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>


namespace xbot::driver::imu {

class ImuDriver {
 private:
  using NotifyHandler = etl::delegate<void(const double*, size_t)>;

  xbot::driver::mcu::Dispatcher* mcu_driver_; 
  mutable struct Data {
    bool valid = false;
    int16_t gyro[3] = {0, 0, 0};
    int16_t accel[3] = {0, 0, 0};
    int16_t orientation[3] = {0, 0, 0};
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

  NotifyHandler registered_handler_{};

 public:

  ImuDriver(xbot::driver::mcu::Dispatcher* dispatcher);
  ~ImuDriver() = default;

  // Initialize hardware / bus
  void Start();

  // Register a callback to be invoked with the latest axes data whenever a GD
  // (gyro) message arrives. Accept the delegate by const-ref so callers can
  // pass temporaries; the driver makes its own copy for storage.
  void RegisterNotifyCallback(const NotifyHandler& handler);

  Data GetData();

 private:
  const double accel_scale_factor = 0.01; //cm/s^2 -> m/s^2
  // 1 LSB = 0.01 rad/s (0.573 deg/s). No longer a guess: verified by replaying a
  // 10 min RTK-fixed recording and sweeping this factor, scoring integrated
  // gyro.z against GPS course. Median heading error bottoms out sharply at this
  // value - 5.7 deg here, versus 11.5 deg at half again and 44 deg at pi/180 -
  // and the minimum holds with GPS heading fusion switched off, so it is the
  // gyro being measured and not the correction loop.
  //
  // Do not re-derive this by regressing against diff_drive/measured_twist: the
  // wheel yaw rate is noisier than the gyro and slip-inflated in exactly the
  // turns that dominate the fit, which biases the estimate high by ~70%.
  const double gyro_scale_factor = 0.01; // rad/s per LSB
  void OnGD(const uint8_t *payload, size_t length, uint8_t ack);
  void OnGF(const uint8_t *payload, size_t length, uint8_t ack);
  void OnGH(const uint8_t *payload, size_t length, uint8_t ack);
  void OnGI(const uint8_t *payload, size_t length, uint8_t ack);
  void OnGS(const uint8_t *payload, size_t length, uint8_t ack);
  void OnOD(const uint8_t *payload, size_t length, uint8_t ack);
};

} // namespace xbot::driver::imu
