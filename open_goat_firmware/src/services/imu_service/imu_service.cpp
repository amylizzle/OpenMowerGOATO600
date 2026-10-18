#include <globals.hpp>
#include <cstdlib>
#include "imu_service.hpp"

using namespace xbot::driver::imu;

bool ImuService::OnStart() {

  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = new ImuDriver(&mcu_dispatcher_driver);
  }
  driver_->RegisterNotifyCallback(etl::delegate<void(const double*, size_t)>::create<ImuService, &ImuService::OnDriverNotify>(*this));
  return true;
}

void ImuService::OnDriverNotify(const double* axes, size_t length) {
  // The driver hands us accel[0..2], gyro[3..5], orientation[6..8] in the raw
  // sensor frame. AxisRemap maps that sensor frame onto the robot frame; it is
  // set from imu/axis_config over the service registers. Encoding is
  // sign * (1|2|3) for +-(X|Y|Z), so entry v selects input |v|-1.
  if (!AxisRemap.valid || AxisRemap.length != 3 || length < 6) {
    SendAxes(axes, length);
    return;
  }

  double remapped[9]{};
  for (size_t i = 0; i < length && i < 9; ++i) {
    remapped[i] = axes[i];
  }

  // Only accel and gyro live in the sensor's xyz frame. Orientation (6..8) is
  // yaw/roll/pitch, a different ordering, so it must not be remapped here.
  for (size_t triple = 0; triple < 2; ++triple) {
    const size_t base = triple * 3;
    for (size_t i = 0; i < 3; ++i) {
      const int8_t v = AxisRemap.value[i];
      const size_t src = static_cast<size_t>(abs(v) - 1);
      if (v == 0 || src > 2) {
        // Malformed remap, pass the data through untouched rather than
        // silently publishing zeros.
        SendAxes(axes, length);
        return;
      }
      remapped[base + i] = (v < 0 ? -1.0 : 1.0) * axes[base + src];
    }
  }

  SendAxes(remapped, length);
}
