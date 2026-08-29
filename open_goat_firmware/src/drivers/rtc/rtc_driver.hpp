
#include <misc_utils.h>
#include <chrono>
#include <string>
#include <limits>
#include <cstdint>
#include <etl/delegate.h>
#include <drivers/mcu/dispatcher.hpp>

namespace xbot::driver::rtc {

class RTCDriver {
  public:    
    RTCDriver(xbot::driver::mcu::Dispatcher* dispatcher);
    ~RTCDriver() = default;
    void Start();
    void Sync();
  private:
    bool sentRA = false;
    xbot::driver::mcu::Dispatcher* mcu_driver_{};
    void OnRC(const uint8_t *payload, size_t length, uint8_t ack);
    void OnRT(const uint8_t *payload, size_t length, uint8_t ack);
    void OnUC(const uint8_t *payload, size_t length, uint8_t ack);
};
}