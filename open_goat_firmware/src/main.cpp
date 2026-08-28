// Start GPS service on /dev/ttyS4 (baudrate 115200) with NMEA

// Start MCU service on /dev/ttyS3 (baudrate 115200) and assign the MCU
// driver to the remaining services with SetDriver.
//
// This file provides a minimal wiring example. Actual UART driver
// instances / board-specific names must be supplied by the board layer.
#include <cstdint>
#include "../../src/open_mower_ros/services/service_ids.h"
#include "../../src/goat_ros/services/service_ids.h"
#include <globals.cpp>


#include <ulog.h>
#include <unistd.h>

#include <cstdio>
#include <mutex>
#include <xbot-service/Io.hpp>
#include "xbot-service/RemoteLogging.hpp"
#include "xbot-service/portable/system.hpp"


int main() {
	xbot::service::system::initSystem();
	xbot::service::Io::start("0.0.0.0");

	xbot::service::startRemoteLogging();
	
	ULOG_INFO("Starting services...");
	ULOG_INFO("Emergency...");
	emergency_service.start();
	ULOG_INFO("Diff drive...");
	diff_drive_service.start();
	ULOG_INFO("Mower...");
	mower_service.start();
	ULOG_INFO("IMU...");
	imu_service.start();
	ULOG_INFO("RTC...");
	rtc_service.start();
	ULOG_INFO("Power...");
	power_service.start();
	ULOG_INFO("GPS...");
	gps_service.start();
	ULOG_INFO("High level...");
	high_level_service.start();
	ULOG_INFO("Screen...");
	screen_service.start();
	ULOG_INFO("Done!");

	mcu_dispatcher_driver.StartDriver("/dev/ttyS3", 115200);
	ULOG_INFO("MCU Dispatcher started!");
	while(1){
		sleep(1);
	}
	ULOG_ERROR("Shutdown!!!!");
}