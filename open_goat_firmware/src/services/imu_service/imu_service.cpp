#include <globals.hpp>
#include "imu_service.hpp"

using namespace xbot::driver::imu;

bool ImuService::OnStart() {

  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = new ImuDriver(&mcu_dispatcher_driver);
  }

  return true;
}

void ImuService::tick() {
  double axes[9]{};

  driver_->ReadAxes(axes, 9);

  SendAxes(axes, 9);
}
