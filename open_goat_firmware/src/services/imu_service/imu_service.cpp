#include <globals.hpp>
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
  SendAxes(axes, length);
}
