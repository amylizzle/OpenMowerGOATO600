// Start GPS service on /dev/ttyS4 (baudrate 115200) with NMEA

// Start MCU service on /dev/ttyS3 (baudrate 115200) and assign the MCU
// driver to the remaining services with SetDriver.
//
// This file provides a minimal wiring example. Actual UART driver
// instances / board-specific names must be supplied by the board layer.

#include <drivers/gps/gps_driver.h>
#include <drivers/mcu/dispatcher.h>

namespace {
// Example helper that would be called from board-specific startup code.
void StartPeripheralDrivers(UARTDriver* gps_uart, UARTDriver* mcu_uart) {
	using namespace xbot::driver::gps;

	// Start GPS driver (protocol selection done by service/ROS config in real firmware)
	// Example: auto gps = ...; gps->StartDriver(gps_uart, 115200);

	// Start MCU dispatcher - this parses framed MCU messages and dispatches them
	// to registered handlers. A real MCU-backed driver layer would implement
	// concrete BMS/Motor/Charger/Input drivers and register them with services
	// via SetDriver(...).
	// Dispatcher dispatcher;
	// dispatcher.StartDriver(mcu_uart, 115200);

	(void)gps_uart;
	(void)mcu_uart;
}

} // namespace

int main() {
    UARTDriver* gpsuart = CreateUARTDriver("/dev/ttyS3", 115200);
    UARTDriver* mcuuart = CreateUARTDriver("/dev/ttyS2", 115200);

    StartPeripheralDrivers(gpsuart, mcuuart);

    return 0;
}