//
// Created by clemens on 10.01.23.
//

#include "gps_driver.h"

#include <etl/algorithm.h>

#include <cmath>
#include "misc_utils.h"

namespace xbot::driver::gps {

void GpsDriver::RawDataInput(uint8_t *data, size_t size) {
  if (!IsRawMode()) return;
  send_raw(data, size);
}

bool GpsDriver::StartDriver(std::string path, int baud) {
  DbgAssert(stopped_, "don't start the driver twice");
  if (!stopped_) {
    return false;
  }
  
  this->rtklink = new SerialDriver("/dev/ttyS2",baud); //TODO should probably make this configurable with a service register
  this->slink = new SerialDriver(path,baud);
 
  LORAInit();
  GNSSConfig();

  stopped_ = false;
  processing_thread_ = createThread(threadHelper, this);

  return true;
}

void GpsDriver::LORAInit() {
  // write the commands to start the RTK system
  for (std::string_view str : { 
    "rtktype rover\r\n", 
    "LORALOWPOWER OFF\r\n", 
    "TRANS ON COM1 COM3\r\n",
    "setsignalprofile mower\r\n",
    "qualitylevel 0\r\n"
   }) {
      this->rtklink->write(std::vector<uint8_t>(str.begin(), str.end()));
  }
}

void GpsDriver::GNSSConfig() {
  // configure streaming the corrections
  for (std::string_view str : { 
    "COM3 115200 N 8 1 IN:RTCM OUT:BYNAV\r\n", //COM3 serial = RTCM correction input
    "interfacemode com3 rtcm bynav\r\n", //COM3 interface = RTCM
    "SETSIGNALPROFILE MOWER\r\n", //signal profile
    "QUALITYLEVEL 0\r\n", //quality level
    "WORKFREQS B1IB2IB3I BEIDOU2\r\n", //enable BD2/BEIDOU2 constellations
    "fix auto\r\n", //automatic RTK fix mode
    "log bestposa ontime 1\r\n", //RTK-fixed position
    "log com1 gpgga ontime 0.1\r\n", //NMEA position (navigation)
    "log com1 bestposa ontime 0.1\r\n", //best-pos ASCII position
    "saveconfig\r\n", //persist config
   }) {
      this->slink->write(std::vector<uint8_t>(str.begin(), str.end()));
  }
}

void GpsDriver::SetStateCallback(const GpsDriver::StateCallback &function) {
  state_callback_ = function;
}

bool GpsDriver::send_raw(const uint8_t *data, size_t size) {
  mutex_.lock();
  this->slink->write(data, size);
  mutex_.unlock();
  return true;
}

void GpsDriver::threadFunc() {
  // uint32_t last_ndtr = 0;
  while (!stopped_) {
    // Wait for data to arrive
    processing_buffer_len_ = this->slink->read(processing_buffer_, RECV_BUFFER_SIZE);
    if (processing_buffer_len_ == -1) {
      ULOG_ERROR("GPS DRIVER FAILED TO READ FROM SERIAL PORT");
      this->slink->close();
      this->slink->open();
    }
    if (processing_buffer_len_ > 0) {
      ProcessBytes(processing_buffer_, processing_buffer_len_);
      if (IsRawMode()) {
        RawDataOutput(processing_buffer_, processing_buffer_len_);
      }
    } else {
      std::this_thread::sleep_for(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(0.01))
            );
    }
    // last_ndtr = 0;
    processing_buffer_len_ = 0;
    processing_done_ = true;
  }
}

void GpsDriver::threadHelper(void *instance) {
  auto *gps_interface = static_cast<GpsDriver *>(instance);
  gps_interface->threadFunc();
}

void GpsDriver::SendRTCM(const uint8_t *data, size_t size) {
  send_raw(data, size);
}

void GpsDriver::TriggerStateCallback() {
  if (state_callback_) {
    state_callback_(gps_state_);
  }
}
}  // namespace xbot::driver::gps
