// Start GPS service on /dev/ttyS4 (baudrate 115200) with NMEA

// Start MCU service on /dev/ttyS3 (baudrate 115200) and assign the MCU
// driver to the remaining services with SetDriver.
//
// This file provides a minimal wiring example. Actual UART driver
// instances / board-specific names must be supplied by the board layer.
#include <cstdint>
#include "../../open_mower_ros/services/service_ids.h"
#include <globals.cpp>


#include <ulog.h>
#include <unistd.h>

#include <cstdio>
#include <mutex>
#include <xbot-service/Io.hpp>
#include "xbot-service/RemoteLogging.hpp"
#include "xbot-service/portable/system.hpp"

#ifdef ULOG_ENABLED
void console_logger(ulog_level_t severity, char* msg, const void* arg) {
	(void) arg;
  	printf("[%s]: %s\n", ulog_level_name(severity), msg);
}
#endif

int main() {
	xbot::service::system::initSystem();
	xbot::service::startRemoteLogging();
	#ifdef ULOG_ENABLED
	ULOG_SUBSCRIBE(console_logger, ULOG_DEBUG_LEVEL);
	#endif

	emergency_service.start();
	//diff_drive.start();
	//mower_service.start();
	//imu_service.start();
	power_service.start();
	gps_service.start();
	//input_service.start();
	high_level_service.start();

	xbot::service::Io::start();

	while (1) {
		sleep(1);
	}
}