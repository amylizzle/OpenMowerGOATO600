#include "globals.hpp"

xbot::driver::mcu::Dispatcher mcu_dispatcher_driver{};

EmergencyService emergency_service{xbot::service_ids::EMERGENCY};
DiffDriveService diff_drive_service{xbot::service_ids::DIFF_DRIVE};
MowerService mower_service{xbot::service_ids::MOWER};
ImuService imu_service{xbot::service_ids::IMU};
PowerService power_service{xbot::service_ids::POWER};
GpsService gps_service{xbot::service_ids::GPS};
HighLevelService high_level_service{xbot::service_ids::HIGH_LEVEL};
ScreenService screen_service{goat::service_ids::SCREEN};
RTCService rtc_service{goat::service_ids::RTC};
