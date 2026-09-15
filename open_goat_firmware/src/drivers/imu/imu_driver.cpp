#include "imu_driver.hpp"
#include "misc_utils.h"
#include <drivers/mcu/dispatcher.hpp>
#include <etl/delegate.h>

namespace xbot::driver::imu {

ImuDriver::ImuDriver(xbot::driver::mcu::Dispatcher* dispatcher) : mcu_driver_(dispatcher)  { 
        if (dispatcher) {
            dispatcher->RegisterHandler(static_cast<uint8_t>('G'), static_cast<uint8_t>('D'),
                                                                    etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<ImuDriver, &ImuDriver::OnGD>(*this));
            dispatcher->RegisterHandler(static_cast<uint8_t>('G'), static_cast<uint8_t>('F'),
                                                                    etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<ImuDriver, &ImuDriver::OnGF>(*this));
            dispatcher->RegisterHandler(static_cast<uint8_t>('G'), static_cast<uint8_t>('H'),
                                                                    etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<ImuDriver, &ImuDriver::OnGH>(*this));
            dispatcher->RegisterHandler(static_cast<uint8_t>('G'), static_cast<uint8_t>('I'),
                                                                    etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<ImuDriver, &ImuDriver::OnGI>(*this));
            dispatcher->RegisterHandler(static_cast<uint8_t>('G'), static_cast<uint8_t>('S'),
                                                                    etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<ImuDriver, &ImuDriver::OnGS>(*this));
            dispatcher->RegisterHandler(static_cast<uint8_t>('O'), static_cast<uint8_t>('D'),
                                                                    etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<ImuDriver, &ImuDriver::OnOD>(*this));
        }
}

void ImuDriver::Start(){
    //there may be a message needs to be sent here? The decomp wasn't totally clear, it looks like it just 
    // sends a GC message with payload 0,1, or 2. Maybe off/on/init? Probably worth playing with
    const uint8_t payload[1] = {0}; // 0 = init, 1 = start, 2 = stop
    mcu_driver_->SendMessage(static_cast<uint8_t>('G'), static_cast<uint8_t>('C'), payload, sizeof(payload));
}

void ImuDriver::RegisterNotifyCallback(const NotifyHandler& handler) {
    registered_handler_ = handler;
}

ImuDriver::Data ImuDriver::GetData() { return data_; }

// Handler for GD -> imu/ImuSensor
void ImuDriver::OnGD(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length == 0) return;
    bool valid = (payload[0] == 0);
    data_.valid = valid;
    if (!valid) {
        data_.gyro[0] = data_.gyro[1] = data_.gyro[2] = 0;
        data_.accel[0] = data_.accel[1] = data_.accel[2] = 0;
        data_.orientation[0] = data_.orientation[1] = data_.orientation[2] = 0;
        data_.ts = 0;
        ULOG_WARNING("INVALID IMU DATA");
        return;
    }
    if (length >= (0x15 + 4)) {
        data_.orientation[0] = ReadI16Le(payload, 1); //yaw
        data_.orientation[1] = ReadI16Le(payload, 3); //roll
        data_.orientation[2] = ReadI16Le(payload, 5); //pitch
        // payload[7] is a duplicate of payload[1]  
        int16_t check = ReadI16Le(payload, 7);
        if (check != data_.orientation[0]) {
            ULOG_WARNING("IMU ORIENTATION CHECK FAILED: %d != %d", check, data_.orientation[0]);
            return;
        }      
        data_.accel[0] = ReadI16Le(payload, 9);
        data_.accel[1] = ReadI16Le(payload, 11);
        data_.accel[2] = ReadI16Le(payload, 13);
        data_.gyro[0] = ReadI16Le(payload, 15);
        data_.gyro[1] = ReadI16Le(payload, 17);
        data_.gyro[2] = ReadI16Le(payload, 19);
        data_.ts = ReadU32Le(payload, 0x15);
    }

    // Any GD (gyro) message triggers a publish of the latest axes. Order:
    // accel[0..2] (scaled), gyro[0..2] (scaled), orientation[0..2].
    double axes[9]{};
    for (size_t i = 0; i < 3; ++i) axes[i] = data_.accel[i] * accel_scale_factor;
    for (size_t i = 0; i < 3; ++i) axes[3 + i] = data_.gyro[i] * gyro_scale_factor;
    for (size_t i = 0; i < 3; ++i) axes[6 + i] = data_.orientation[i];
    if (registered_handler_) {
        registered_handler_(axes, 9);
    }
}

// Handler for GF -> imu/GyroBias
void ImuDriver::OnGF(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length == 0) return;
    bool valid = (payload[0] == 0);
    if (!valid) {
        for (int i = 0; i < 6; ++i) data_.bias[i] = 0;
        return;
    }
    if (length >= 3) data_.bias[0] = ReadU16Le(payload, 1);
    if (length >= 5) data_.bias[1] = ReadU16Le(payload, 3);
    if (length >= 7) data_.bias[2] = ReadU16Le(payload, 5);
    if (length >= 9) data_.bias[3] = ReadU16Le(payload, 7);
    if (length >= 11) data_.bias[4] = ReadU16Le(payload, 9);
    if (length >= 13) data_.bias[5] = ReadU16Le(payload, 11);
}

// Handler for GH -> imu/geomag (validity + u16,u16,u8)
void ImuDriver::OnGH(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length == 0) return;
    bool valid = (payload[0] == 0);
    if (!valid) {
        data_.geomag_u16[0] = data_.geomag_u16[1] = 0;
        data_.geomag_u8 = 0;
        return;
    }
    if (length >= 3) data_.geomag_u16[0] = ReadU16Le(payload, 1);
    if (length >= 5) data_.geomag_u16[1] = ReadU16Le(payload, 3);
    if (length >= 6) data_.geomag_u8 = payload[5];
}

// Handler for GI -> imu/Geomag (no validity; u16 @0,2,4)
void ImuDriver::OnGI(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length == 0) return;
    if (length >= 2) data_.geomag3[0] = ReadU16Le(payload, 0);
    if (length >= 4) data_.geomag3[1] = ReadU16Le(payload, 2);
    if (length >= 6) data_.geomag3[2] = ReadU16Le(payload, 4);
}

// Handler for GS -> state/status [state, value]
void ImuDriver::OnGS(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length == 0) return;
    data_.state = payload[0];
    data_.state_value = (length > 1) ? payload[1] : 0;
}

// Handler for OD -> pairs [type,value]
void ImuDriver::OnOD(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length < 2) return;
    for (size_t i = 0; i + 1 < length; i += 2) {
        uint8_t t = payload[i];
        uint8_t v = payload[i+1];
        if (t == 0) data_.ultrasonic = v;
        else if (t == 1) data_.gyro_type = v;
    }
}
}
