// Start GPS service on /dev/ttyS4 (baudrate 115200) with NMEA

// Start MCU service on /dev/ttyS3 (baudrate 115200) and assign the MCU
// driver to the remaining services with SetDriver.
//
// This file provides a minimal wiring example. Actual UART driver
// instances / board-specific names must be supplied by the board layer.
#include <cstdint>
#include "../../src/open_mower_ros/services/service_ids.h"
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
	xbot::service::Io::start("0.0.0.0");

	xbot::service::startRemoteLogging();
	#ifdef ULOG_ENABLED
	ULOG_SUBSCRIBE(console_logger, ULOG_DEBUG_LEVEL);
	#endif

	UARTDriver* mcu_uart = CreateUARTDriver("/dev/ttyM1", 115200);
	if (mcu_uart == nullptr){
		ULOG_ERROR("Failed to get /dev/ttyM1 for MCU!");
		exit(1);
	}
	
	ULOG_INFO("Starting services...");
	ULOG_INFO("Emergency...");
	emergency_service.start();
	ULOG_INFO("Diff drive...");
	diff_drive.start();
	ULOG_INFO("Mower...");
	mower_service.start();
	ULOG_INFO("IMU...");
	imu_service.start();
	ULOG_INFO("Power...");
	power_service.start();
	ULOG_INFO("GPS...");
	gps_service.start();
	ULOG_INFO("High level...");
	high_level_service.start();
	ULOG_INFO("Done!");

	mcu_dispatcher_driver.StartDriver(mcu_uart);
	ULOG_INFO("MCU Dispatcher started!");
	while(1){
		sleep(1);
	}
	ULOG_ERROR("Shutdown!!!!");
}