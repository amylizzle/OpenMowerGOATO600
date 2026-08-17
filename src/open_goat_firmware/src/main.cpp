// Start GPS service on /dev/ttyS4 (baudrate 115200) with NMEA

// Start MCU service on /dev/ttyS3 (baudrate 115200) and assign the MCU
// driver to the remaining services with SetDriver.
//
// This file provides a minimal wiring example. Actual UART driver
// instances / board-specific names must be supplied by the board layer.

#include "service_ids.h"
#include <globals.cpp>

int main() {
    return 0;
}