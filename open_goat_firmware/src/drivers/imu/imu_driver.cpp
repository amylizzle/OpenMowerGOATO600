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
    //sets scale to 1.0 with a UC message, possibly there is some time dependant scaling going on?
}

void ImuDriver::ReadAxes(double* axes, size_t length){
        // fill axes buffer with latest accel, gyro, mag values where available
        if (!axes || length == 0) return;
        // order: accel[0..2], gyro[0..w], mag[0..2]
        if (length >= 1) {
            size_t i = 0;
            for (; i < length && i < 3; ++i) axes[i] = static_cast<double>(data_.accel[i]) * accel_scale_factor;
            for (size_t g = 0; i < length && g < 3; ++g, ++i) axes[i] = data_.gyro[g] * gyro_scale_factor;
            for (size_t m = 0; i < length && m < 3; ++m, ++i) axes[i] = static_cast<double>(data_.mag[m]);
            // fill remaining with zeros
            for (; i < length; ++i) axes[i] = 0.0;
        }
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



struct Vec3 { float x, y, z; };
struct Quat { float w, x, y, z; };

// Converts 2 consecutive quaternions into angular velocity (rad/s)
Vec3 quatToAngularVel(Quat q1, Quat q2, float dt) {
    // Relative quaternion: dq = inv(q1) * q2
    Quat dq = {
        q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z,
        q1.w*q2.x - q1.x*q2.w - q1.y*q2.z + q1.z*q2.y,
        q1.w*q2.y + q1.x*q2.z - q1.y*q2.w - q1.z*q2.x,
        q1.w*q2.z - q1.x*q2.y + q1.y*q2.x - q1.z*q2.w
    };
    if (dq.w < 0.0f) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; } // Shortest path
    return { (2.0f * dq.x) / dt, (2.0f * dq.y) / dt, (2.0f * dq.z) / dt };
}

void normaliseQuat(Quat& q) {
    float invLen = 1.0f / std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w *= invLen; q.x *= invLen; q.y *= invLen; q.z *= invLen;
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
        return;
    }
    if (length >= (0x15 + 4)) {
        Quat q1 = {static_cast<float>(data_.gyro_quat[0]),
                   static_cast<float>(data_.gyro_quat[1]), 
                   static_cast<float>(data_.gyro_quat[2]), 
                   static_cast<float>(data_.gyro_quat[3])};
        data_.gyro_quat[0] = read_i16_le(payload, 1);
        data_.gyro_quat[1] = read_i16_le(payload, 3);
        data_.gyro_quat[2] = read_i16_le(payload, 5);
        data_.gyro_quat[3] = read_i16_le(payload, 7);
        Quat q2 = {static_cast<float>(data_.gyro_quat[0]),
                   static_cast<float>(data_.gyro_quat[1]), 
                   static_cast<float>(data_.gyro_quat[2]), 
                   static_cast<float>(data_.gyro_quat[3])};      
        
        data_.accel[0] = read_i16_le(payload, 9);
        data_.accel[1] = read_i16_le(payload, 11);
        data_.accel[2] = read_i16_le(payload, 13);
        data_.mag[0] = read_i16_le(payload, 15);
        data_.mag[1] = read_i16_le(payload, 17);
        data_.mag[2] = read_i16_le(payload, 19);
        uint32_t dt = data_.ts;
        data_.ts = read_u32_le(payload, 0x15);
        if(dt > 0){
            dt = data_.ts - dt;
            //normalise the quats
            normaliseQuat(q1);
            normaliseQuat(q2);
            Vec3 omega = quatToAngularVel(q1,q2,dt/1000.0f);
            data_.gyro[0] = omega.x;
            data_.gyro[1] = omega.y;
            data_.gyro[2] = omega.z;
        }
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
