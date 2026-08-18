// Minimal globals.hpp stub to satisfy includes during POSIX build
#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "services/diff_drive_service/diff_drive_service.hpp"
#include "services/emergency_service/emergency_service.hpp"
#include "services/gps_service/gps_service.hpp"
#include "services/high_level_service/high_level_service.hpp"
#include "services/imu_service/imu_service.hpp"
#include "services/mower_service/mower_service.hpp"
#include "services/power_service/power_service.hpp"

extern EmergencyService emergency_service;
extern DiffDriveService diff_drive;
extern MowerService mower_service;
extern ImuService imu_service;
extern PowerService power_service;
extern GpsService gps_service;
extern HighLevelService high_level_service;

#endif // GLOBALS_HPP
