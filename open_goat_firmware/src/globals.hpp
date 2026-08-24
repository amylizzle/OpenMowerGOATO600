// Minimal globals.hpp stub to satisfy includes during POSIX build
#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "drivers/mcu/dispatcher.hpp"

#include "services/diff_drive_service/diff_drive_service.hpp"
#include "services/emergency_service/emergency_service.hpp"
#include "services/gps_service/gps_service.hpp"
#include "services/high_level_service/high_level_service.hpp"
#include "services/imu_service/imu_service.hpp"
#include "services/mower_service/mower_service.hpp"
#include "services/power_service/power_service.hpp"
#include "services/screen_service/screen_service.hpp"

extern xbot::driver::mcu::Dispatcher mcu_dispatcher_driver;

extern EmergencyService emergency_service;
extern DiffDriveService diff_drive_service;
extern MowerService mower_service;
extern ImuService imu_service;
extern PowerService power_service;
extern GpsService gps_service;
extern HighLevelService high_level_service;
extern ScreenService screen_service;

#endif // GLOBALS_HPP
