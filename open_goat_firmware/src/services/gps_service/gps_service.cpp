#include "gps_service.hpp"

#include <drivers/gps/nmea_gps_driver.h>
#include <drivers/gps/ublox_gps_driver.h>
#include <ulog.h>

#include <cstdio>
#include "misc_utils.h"

bool GpsService::LoadAndStartGpsDriver(ProtocolType protocol_type, const char *device, uint32_t baudrate) {
  // Create the requested driver
  if (protocol_type == ProtocolType::UBX) {
    gps_driver_ = new UbxGpsDriver();
  } else {
    gps_driver_ = new NmeaGpsDriver();
  }

  gps_driver_->SetStateCallback(
      etl::delegate<void(const GpsDriver::GpsState&)>::create<GpsService, &GpsService::GpsStateCallback>(*this));

  gps_driver_->StartDriver(device, baudrate);

  return true;
}

bool GpsService::OnStart() {
  using namespace xbot::driver::gps;
  ULOG_WARNING("GPS SERVICE STARTED!");
  if (gps_driver_ == nullptr) {
    // We don't have a gps driver running yet, so create one.
    ULOG_WARNING(("Starting GPS driver on /dev/ttyS" + std::to_string(Uart.value)).c_str());
    return LoadAndStartGpsDriver(Protocol.value, ("/dev/ttyS" + std::to_string(Uart.value)).c_str(), Baudrate.value);
  }

  return true;
}

void GpsService::OnRTCMChanged(const uint8_t* new_value, uint32_t length) {
  // Update NTRIP timestamp when RTCM data is received
  last_ntrip_time_ = chVTGetSystemTimeX();

  gps_driver_->SendRTCM(new_value, length);
}

void GpsService::GpsStateCallback(const GpsDriver::GpsState& state) {
  StartTransaction();
  double position[3] = {state.pos_lat, state.pos_lon, state.pos_height};
  SendPosition(position, 3);
  SendPositionHorizontalAccuracy(state.position_h_accuracy);
  SendPositionVerticalAccuracy(state.position_v_accuracy);
  if (state.rtk_type == xbot::driver::gps::GpsDriver::GpsState::RTK_FIX) {
    SendFixType("FIX", 3);
  } else if (state.rtk_type == xbot::driver::gps::GpsDriver::GpsState::RTK_FLOAT) {
    SendFixType("FLOAT", 5);
  }
  double vel[3] = {state.vel_e, state.vel_n, state.vel_u};
  SendMotionVectorENU(vel, 3);
  CommitTransaction();
}

uint32_t GpsService::GetSecondsSinceLastRtcmPacket() const {
  if (last_ntrip_time_ == 0) {
    return 0;  // No RTCM data received yet
  }
  return TIME_I2S(chVTGetSystemTimeX() - last_ntrip_time_);
}
