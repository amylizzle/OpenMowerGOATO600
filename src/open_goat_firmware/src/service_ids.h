#ifndef SERVICE_IDS_H
#define SERVICE_IDS_H
//just copied from open_mower_ros and slightly edited because I could not make it compile no matter what I did
namespace xbot::service_ids {
enum {
  EMERGENCY = 1,
  DIFF_DRIVE = 2,
  MOWER = 3,
  IMU = 4,
  POWER = 5,
  GPS = 6,
  INPUT = 7,
  HIGH_LEVEL = 8,
  BMS = 9,
  REMOTE_GPIO = 10,
  META = 11
};
}

#endif  // SERVICE_IDS_H
