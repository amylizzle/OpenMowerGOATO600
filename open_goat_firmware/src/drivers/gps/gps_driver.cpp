//
// Created by clemens on 10.01.23.
//

#include "gps_driver.h"

#include <etl/algorithm.h>

#include <cmath>
#include "posix_ch.h"

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
  
  this->slink = new SerialDriver(path,baud);

  stopped_ = false;
  processing_thread_ = createThread(threadHelper, this);

  return true;
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

    if (processing_buffer_len_ > 0) {
      ProcessBytes(processing_buffer_, processing_buffer_len_);
      if (IsRawMode()) {
        RawDataOutput(processing_buffer_, processing_buffer_len_);
      }
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
