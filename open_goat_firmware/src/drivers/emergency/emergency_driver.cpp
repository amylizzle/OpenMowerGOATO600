#include "emergency_driver.hpp"
#include <EmergencyServiceBase.hpp>
#include <ulog.h>

namespace xbot::driver::emergency{

EmergencyDriver::EmergencyDriver(xbot::driver::mcu::Dispatcher* dispatcher) : mcu_driver_(dispatcher)  { 
    dispatcher->RegisterHandler(
          static_cast<uint8_t>('B'), static_cast<uint8_t>('C'), // GPIO sensor status message
          etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<EmergencyDriver, &EmergencyDriver::OnBCMessage>(*this));
}

uint16_t EmergencyDriver::GetEmergencyState() {
    uint16_t state = 0;
    if(this->bump) state |= EmergencyReason::COLLISION;
    if(this->fall) state |= EmergencyReason::LIFT;
    if(this->roll) state |= EmergencyReason::LIFT_MULTIPLE;
    if(this->Stop) state |= EmergencyReason::STOP;
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
                ULOG_WARNING("unexpected GPIO sensor value"); break;
        }
    }
    if(change){
        if (registered_handler_)
            registered_handler_(GetEmergencyState());
    }
}

void EmergencyDriver::RegisterNotifyCallback(const NotifyHandler& handler) {
    registered_handler_ = handler;
}

void EmergencyDriver::UpdateEmergency(uint16_t /*add*/, uint16_t /*clear*/){}

}