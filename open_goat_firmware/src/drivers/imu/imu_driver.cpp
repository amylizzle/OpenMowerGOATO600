#include "imu_driver.hpp"
#include <drivers/mcu/dispatcher.hpp>
#include <etl/delegate.h>

namespace xbot::driver::imu {

ImuDriver::ImuDriver(xbot::driver::mcu::Dispatcher* dispatcher) : mcu_driver_(dispatcher)  { 
    // dispatcher->RegisterHandler(
    //       static_cast<uint8_t>('G'), static_cast<uint8_t>('D'), // GPIO sensor status message
    //       etl::delegate<void(const uint8_t *, size_t)>::create<EmergencyDriver, &EmergencyDriver::OnGDMessage>(*this));
}

void ImuDriver::Start(){
    //there may be a message needs to be sent here? The decomp wasn't totally clear, it looks like it just 
    //sets scale to 1.0 with a UC message, possibly there is some time dependant scaling going on?
}

void ImuDriver::ReadAxes(double* axes, size_t length){
    axes[length-1] = 1.0;
}
};
//   def parse_GD(self, data, ack):
//         """GD -> imu/ImuSensor. data[0]=validity (must 0); i16 @1,3,5,7 (scaled
//         by gyro scale), i16 @9,0xb,0xd (3 floats), i16 @0xf,0x11,0x13 (3 floats),
//         i32 @0x15 timestamp/counter."""
//         r = {"ack": ack, "valid": data[0] if data else 1}
//         if not r["valid"]:
//             r["gyro"] = [_s16(data, i) for i in (1, 3, 5, 7)]
//             r["accel"] = [_s16(data, i) for i in (9, 0xB, 0xD)]
//             r["mag"]   = [_s16(data, i) for i in (0xF, 0x11, 0x13)]
//             if len(data) > 0x15:
//                 r["ts"] = _s32(data, 0x15)
//         return r

//     def parse_GF(self, data, ack):
//         """GF -> imu/GyroBias: validity data[0] must 0; u16 @1,3,5,7 (3 floats)
//         and @9,0xb (2 floats)."""
//         r = {"ack": ack, "valid": data[0] if data else 1}
//         if not r["valid"]:
//             r["bias"] = [_u16(data, i) for i in (1, 3, 5, 7, 9, 0xB)]
//         return r

//     def parse_GH(self, data, ack):
//         """GH -> imu/geomag: validity data[0] must 0; u16@1,@3, u8@5."""
//         r = {"ack": ack, "valid": data[0] if data else 1}
//         if not r["valid"]:
//             r["geomag"] = [_u16(data, 1), _u16(data, 3), data[5]]
//         return r

//     def parse_GI(self, data, ack):
//         """GI -> imu/Geomag: no validity; u16 @0,2,4."""
//         return {"ack": ack, "geomag": [_u16(data, i) for i in (0, 2, 4)]}

//     def parse_GS(self, data, ack):
//         """GS state/status: 2-byte payload [state, value] -> ImuState."""
//         return {"ack": ack,
//                 "state": data[0] if data else 0,
//                 "value": data[1] if len(data) > 1 else 0}

//     def parse_OD(self, data, ack):
//         """OD: [type,value] byte pairs; type 0->UltraSonicState, type 1->GyroType."""
//         out = {}
//         for i in range(0, len(data) - 1, 2):
//             t, v = data[i], data[i+1]
//             if t == 0:
//                 out["ultrasonic"] = v
//             elif t == 1:
//                 out["gyro_type"] = v
//         out["ack"] = ack
//         return out