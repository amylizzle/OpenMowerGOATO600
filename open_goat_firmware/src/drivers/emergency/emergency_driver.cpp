#include "emergency_driver.hpp"
#include <EmergencyServiceBase.hpp>
#include <ulog.h>

namespace xbot::driver::emergency{

EmergencyDriver::EmergencyDriver(xbot::driver::mcu::Dispatcher* dispatcher) : mcu_driver_(dispatcher)  { 
    dispatcher->RegisterHandler(
          static_cast<uint8_t>('B'), static_cast<uint8_t>('C'), // GPIO sensor status message
          etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<EmergencyDriver, &EmergencyDriver::OnBCMessage>(*this));
    dispatcher->RegisterHandler( // DB: UrgentAlarm / MCU alert report (mcuAlarmCode + motorFaultCode).
          static_cast<uint8_t>('D'), static_cast<uint8_t>('B'),
          etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<EmergencyDriver, &EmergencyDriver::OnDBMessage>(*this));
}

uint16_t EmergencyDriver::GetEmergencyState() {
    uint16_t state = 0;
    if(this->bump > 64) state |= EmergencyReason::COLLISION;
    if(this->fall > 64) state |= EmergencyReason::LIFT;
    if(this->roll) state |= EmergencyReason::LIFT_MULTIPLE;
    if(this->Stop) state |= EmergencyReason::STOP;
    // DB 
    state |= AlarmBitsToEmergencyReason(this->mcuAlarmCode);
    // A non-zero motor fault (any of the 4 motor err codes) is an emergency too.
    if (this->motorFaultCode != 0 || this->mcuAlarmCode != 0) {
        state |= EmergencyReason::LIFT_MULTIPLE;
    }
    
    return state;
}

void EmergencyDriver::OnBCMessage(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    bool change = false;
    // Loop through the data in steps of 2, first byte is index, second is value
    for (size_t i = 0; i + 1 < length; i += 2) {
        uint8_t idx = payload[i];
        uint8_t state = payload[i + 1];
        
        // Store values in variables based on the index
        switch (idx) {
            case 0:  change |= (this->bump != state); this->bump = state; break;
            case 2:  change |= (this->fall != state); this->fall = state; break;
            case 4:  change |= (this->chargeState != state); this->chargeState = state; break;
            case 6:  change |= (this->acczero != state); this->acczero = state; break;
            case 8:  change |= (this->rain != state); this->rain = state; break;
            case 10: change |= (this->grass != state); this->grass = state; break;
            case 12: change |= (this->roll != state); this->roll = state; break;
            case 14: change |= (this->Stop != state); this->Stop = state; break;
            case 16: change |= (this->fan != state); this->fan = state; break;
            default: 
                ULOG_WARNING("unexpected GPIO sensor value %u, %u", idx, state); break;
        }
    }
    if(change){
        ULOG_ERROR("EMERGENCY STATE CHANGE: bump: %u, fall: %u, charge: %u, acczero: %u, rain: %u, grass: %u, roll: %u, stop: %u, fan: %u", this->bump, this->fall, this->chargeState, this->acczero, this->rain, this->grass, this->roll, this->Stop, this->fan);
        if (registered_handler_)
            registered_handler_(GetEmergencyState());
    }
}

void EmergencyDriver::RegisterNotifyCallback(const NotifyHandler& handler) {
    registered_handler_ = handler;
}

void EmergencyDriver::ClearEStop(){
    //send a JA message with a 16bit 0
    ULOG_INFO("EMERGENCY: ClearEStop() sending JA 0 message to MCU");
    mcu_driver_->SendMessage('J', 'A', (uint16_t)0, 2);
}

// Map ECOVACS mcuAlarmCode EMERGENCY_* bits onto the host EmergencyReason mask.
// Only the bits that have a direct host counterpart are forwarded.
uint16_t EmergencyDriver::AlarmBitsToEmergencyReason(uint32_t mcuAlarmCode) {
    uint16_t state = 0;
    if (mcuAlarmCode & EMERGENCY_BUMP)      state |= EmergencyReason::COLLISION;
    if (mcuAlarmCode & EMERGENCY_INCLINE)   state |= EmergencyReason::COLLISION;
    if (mcuAlarmCode & EMERGENCY_ELEVATE)   state |= EmergencyReason::LIFT;
    if (mcuAlarmCode & EMERGENCY_TURNOVER)  state |= EmergencyReason::LIFT;
    if (mcuAlarmCode & EMERGENCY_LIFTMOT)   state |= EmergencyReason::LIFT_MULTIPLE;
    if (mcuAlarmCode & EMERGENCY_STOP)      state |= EmergencyReason::STOP;
    return state;
}

// DB: UrgentAlarm / MCU alert report.
//   off 0-3   u32 mcuAlarmCode     (EMERGENCY_* bitmask)
//   off 4-7   u32 motorFaultCode   (4 u8 motor err codes)
//   off 8-9   u16 liftFaultCode    (only if length > 9)
//   off 10-11 u16 grassFaultCode   (only if length > 11)
void EmergencyDriver::OnDBMessage(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
    if (!payload) {
        return;
    }

    bool anyChange = false;
    bool motorChange = false;

    // mcuAlarmCode (u32 LE at off 0)
    if (length >= 4) {
        uint32_t code = static_cast<uint32_t>(payload[0]) |
                        (static_cast<uint32_t>(payload[1]) << 8) |
                        (static_cast<uint32_t>(payload[2]) << 16) |
                        (static_cast<uint32_t>(payload[3]) << 24);
        if (code != this->mcuAlarmCode) {
            this->mcuAlarmCode = code;
            ULOG_INFO("EMERGENCY STATE CHANGE: mcuAlarmCode:0x%08x", code);
            anyChange = true;
        }
    }

    // motorFaultCode (u32 LE at off 4)
    if (length >= 8) {
        uint32_t code = static_cast<uint32_t>(payload[4]) |
                        (static_cast<uint32_t>(payload[5]) << 8) |
                        (static_cast<uint32_t>(payload[6]) << 16) |
                        (static_cast<uint32_t>(payload[7]) << 24);
        if (code != this->motorFaultCode) {
            this->motorFaultCode = code;
            uint8_t l_motor = static_cast<uint8_t>(code & 0xFFu);
            uint8_t r_motor = static_cast<uint8_t>((code >> 8) & 0xFFu);
            uint8_t l_cut   = static_cast<uint8_t>((code >> 16) & 0xFFu);
            uint8_t r_cut   = static_cast<uint8_t>((code >> 24) & 0xFFu);
            ULOG_INFO("EMERGENCY STATE CHANGE: motorFaultCode:0x%08x l_motor_err_code=0x%02x r_motor_err_code=0x%02x l_cut_err_code=0x%02x r_cut_err_code=0x%02x",
                      code, l_motor, r_motor, l_cut, r_cut);
            anyChange = true;
            motorChange = true;
        }
    }

    // liftFaultCode (u16 LE at off 8)
    if (length > 9) {
        uint16_t code = static_cast<uint16_t>(payload[8]) |
                        (static_cast<uint16_t>(payload[9]) << 8);
        if (code != this->liftFaultCode) {
            this->liftFaultCode = code;
            ULOG_INFO("EMERGENCY STATE CHANGE: liftFaultCode:0x%04x", code);
            anyChange = true;
        }
    }

    // grassFaultCode (u16 LE at off 10)
    if (length > 11) {
        uint16_t code = static_cast<uint16_t>(payload[10]) |
                        (static_cast<uint16_t>(payload[11]) << 8);
        if (code != this->grassFaultCode) {
            this->grassFaultCode = code;
            ULOG_INFO("EMERGENCY STATE CHANGE: grassFaultCode:0x%04x", code);
            anyChange = true;
        }
    }

    // If all four fields are zero this is the "no alert" report; the host
    // emergency state is cleared by the mapped reason bits going to zero.
    (void) motorChange;

    if (anyChange) {
        if (registered_handler_) {
            registered_handler_(GetEmergencyState());
        }
    }
}

}