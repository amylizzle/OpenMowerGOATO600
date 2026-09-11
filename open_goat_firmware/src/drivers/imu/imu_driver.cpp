#include "imu_driver.hpp"
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

static inline int16_t read_i16_le(const uint8_t* b, size_t idx) {
    return static_cast<int16_t>(static_cast<uint16_t>(b[idx]) | (static_cast<uint16_t>(b[idx+1]) << 8));
}

static inline uint32_t read_u32_le(const uint8_t* b, size_t idx) {
    return static_cast<uint32_t>(static_cast<uint32_t>(b[idx]) | (static_cast<uint32_t>(b[idx+1]) << 8) |
                                                             (static_cast<uint32_t>(b[idx+2]) << 16) | (static_cast<uint32_t>(b[idx+3]) << 24));
}

static inline uint16_t read_u16_le(const uint8_t* b, size_t idx) {
    return static_cast<uint16_t>(static_cast<uint16_t>(b[idx]) | (static_cast<uint16_t>(b[idx+1]) << 8));
}


// Handler for GD -> imu/ImuSensor
void ImuDriver::OnGD(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length == 0) return;
    bool valid = (payload[0] == 0);
    data_.valid = valid;
    if (!valid) {
        data_.gyro[0] = data_.gyro[1] = data_.gyro[2] = 0;
        data_.accel[0] = data_.accel[1] = data_.accel[2] = 0;
        data_.mag[0] = data_.mag[1] = data_.mag[2] = 0;
        data_.ts = 0;
        ULOG_WARNING("INVALID IMU DATA");
        return;
    }
    if (length >= (0x15 + 4)) {
        data_.prevgyro[0] = data_.rawgyro[0];
        data_.prevgyro[1] = data_.rawgyro[1];
        data_.prevgyro[2] = data_.rawgyro[2];
        data_.prevaccel[0] = data_.rawaccel[0];
        data_.prevaccel[1] = data_.rawaccel[1];
        data_.prevaccel[2] = data_.rawaccel[2];
        data_.prevmag[0] = data_.rawmag[0];
        data_.prevmag[1] = data_.rawmag[1];
        data_.prevmag[2] = data_.rawmag[2];
        data_.prevts = data_.ts;            
        data_.rawgyro[0] = read_i16_le(payload, 1); //yaw
        data_.rawgyro[1] = read_i16_le(payload, 3); //roll
        data_.rawgyro[2] = read_i16_le(payload, 5); //pitch
        // payload[7] is a duplicate of payload[1]        
        data_.rawaccel[0] = read_i16_le(payload, 9);
        data_.rawaccel[1] = read_i16_le(payload, 11);
        data_.rawaccel[2] = read_i16_le(payload, 13);
        data_.rawmag[0] = read_i16_le(payload, 15);
        data_.rawmag[1] = read_i16_le(payload, 17);
        data_.rawmag[2] = read_i16_le(payload, 19);
        data_.ts = read_u32_le(payload, 0x15);
    }

    data_.gyro[0] = static_cast<float>(data_.rawgyro[0] - data_.prevgyro[0])/(data_.ts - data_.prevts);
    data_.gyro[1] = static_cast<float>(data_.rawgyro[1] - data_.prevgyro[1])/(data_.ts - data_.prevts);
    data_.gyro[2] = static_cast<float>(data_.rawgyro[2] - data_.prevgyro[2])/(data_.ts - data_.prevts);
    data_.accel[0] = static_cast<float>(data_.rawaccel[0] - data_.prevaccel[0])/(data_.ts - data_.prevts);
    data_.accel[1] = static_cast<float>(data_.rawaccel[1] - data_.prevaccel[1])/(data_.ts - data_.prevts);
    data_.accel[2] = static_cast<float>(data_.rawaccel[2] - data_.prevaccel[2])/(data_.ts - data_.prevts);
    data_.mag[0] = static_cast<float>(data_.rawmag[0] - data_.prevmag[0])/(data_.ts - data_.prevts);
    data_.mag[1] = static_cast<float>(data_.rawmag[1] - data_.prevmag[1])/(data_.ts - data_.prevts);
    data_.mag[2] = static_cast<float>(data_.rawmag[2] - data_.prevmag[2])/(data_.ts - data_.prevts);    
    
    // Any GD (gyro) message triggers a publish of the latest axes. Order:
    // accel[0..2] (scaled), gyro[0..2] (scaled), mag[0..2].
    double axes[9]{};
    for (size_t i = 0; i < 3; ++i) axes[i] = static_cast<double>(data_.accel[i]) * accel_scale_factor;
    for (size_t i = 0; i < 3; ++i) axes[3 + i] = data_.gyro[i] * gyro_scale_factor;
    for (size_t i = 0; i < 3; ++i) axes[6 + i] = static_cast<double>(data_.mag[i]);
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
    if (length >= 3) data_.bias[0] = read_u16_le(payload, 1);
    if (length >= 5) data_.bias[1] = read_u16_le(payload, 3);
    if (length >= 7) data_.bias[2] = read_u16_le(payload, 5);
    if (length >= 9) data_.bias[3] = read_u16_le(payload, 7);
    if (length >= 11) data_.bias[4] = read_u16_le(payload, 9);
    if (length >= 13) data_.bias[5] = read_u16_le(payload, 11);
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
    if (length >= 3) data_.geomag_u16[0] = read_u16_le(payload, 1);
    if (length >= 5) data_.geomag_u16[1] = read_u16_le(payload, 3);
    if (length >= 6) data_.geomag_u8 = payload[5];
}

// Handler for GI -> imu/Geomag (no validity; u16 @0,2,4)
void ImuDriver::OnGI(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload || length == 0) return;
    if (length >= 2) data_.geomag3[0] = read_u16_le(payload, 0);
    if (length >= 4) data_.geomag3[1] = read_u16_le(payload, 2);
    if (length >= 6) data_.geomag3[2] = read_u16_le(payload, 4);
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
