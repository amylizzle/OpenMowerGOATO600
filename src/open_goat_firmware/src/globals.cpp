#include "globals.hpp"

#include "services/bms_service/bms_service.hpp"
// #include "services/diff_drive_service/diff_drive_service.hpp"
#include "services/emergency_service/emergency_service.hpp"
#include "services/gps_service/gps_service.hpp"
#include "services/high_level_service/high_level_service.hpp"
// #include "services/imu_service/imu_service.hpp"
// #include "services/input_service/input_service.hpp"
// #include "services/mower_service/mower_service.hpp"
#include "services/power_service/power_service.hpp"

EmergencyService emergency_service{xbot::service_ids::EMERGENCY};
// DiffDriveService diff_drive{xbot::service_ids::DIFF_DRIVE};
// MowerService mower_service{xbot::service_ids::MOWER};
// ImuService imu_service{xbot::service_ids::IMU};
BmsService bms_service{xbot::service_ids::BMS};
PowerService power_service{xbot::service_ids::POWER};
GpsService gps_service{xbot::service_ids::GPS};
// InputService input_service{xbot::service_ids::INPUT};
HighLevelService high_level_service{xbot::service_ids::HIGH_LEVEL};
