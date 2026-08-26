//
// Created by clemens on 10.01.23.
//

#ifndef XBOT_DRIVER_GPS_GPS_DRIVER_H
#define XBOT_DRIVER_GPS_GPS_DRIVER_H

#include <etl/delegate.h>

#include "GpsServiceBase.hpp"
#include "misc_utils.h"
#include <mutex>
#include "SerialDriver.cpp"

namespace xbot::driver::gps {
class GpsDriver {
 public:
  void RawDataInput(uint8_t *data, size_t size);

  ~GpsDriver() = default;

  /*
   * The final GPS state we're interested in.
   */
  struct GpsState {
    enum FixType { NO_FIX = 0, DR_ONLY = 1, FIX_2D = 2, FIX_3D = 3, GNSS_DR_COMBINED = 4 };

    enum RTKType { RTK_NONE = 0, RTK_FLOAT = 1, RTK_FIX = 2 };

    uint32_t sensor_time;
    uint32_t received_time;

    // Position
    bool position_valid;
    // Position accuracy in m
    double position_h_accuracy;
    double position_v_accuracy;
    double pos_lat, pos_lon, pos_height;

    // Motion
    bool motion_heading_valid;
    double vel_e, vel_n, vel_u;
    double motion_heading_accuracy;
    double motion_heading;

    // Heading
    bool vehicle_heading_valid;
    double vehicle_heading_accuracy;
    // Vehicle heading in rad.
    double vehicle_heading;

    FixType fix_type;
    RTKType rtk_type;

    // Number of satellites used in solution
    uint8_t num_sv;
  };

  enum Level { VERBOSE, INFO, WARN, ERROR };

  typedef etl::delegate<void(const GpsState &new_state)> StateCallback;

 public:
  bool StartDriver(std::string path, int baud);
  void SetStateCallback(const GpsDriver::StateCallback &function);

  void SendRTCM(const uint8_t *data, size_t size);
  void GNSSConfig();
  void LORAInit();
  /**
   * @brief Get current GPS state
   * @return Current GPS state
   */
  const GpsState &GetGpsState() const {
    return gps_state_;
  }

  /**
   * @brief Check if GPS state is valid
   * @return true if GPS state is valid
   */
  bool IsGpsStateValid() const {
    return gps_state_valid_;
  }

  virtual ProtocolType GetProtocolType() const = 0;

  // Optional: when in raw mode, drivers may emit raw output
  virtual bool IsRawMode() const { return false; }
  virtual void RawDataOutput(const uint8_t *data, size_t size) { (void)data; (void)size; }


 protected:
  StateCallback state_callback_{};
  void TriggerStateCallback();

  bool gps_state_valid_{};
  GpsState gps_state_{};

  /**
   * Send a message to the GPS. This will just output to the serial port
   * directly
   */
  bool send_raw(const uint8_t *data, size_t size);

  // Called on serial reconnect
  virtual void ResetParserState() = 0;

  virtual size_t ProcessBytes(const uint8_t *buffer, size_t len) = 0;

 private:

  static constexpr size_t RECV_BUFFER_SIZE = 512;
  // 20Hz timeout for reception
  static constexpr uint32_t RECV_TIMEOUT_MILLIS = 25;
  // Keep two buffers for streaming data while doing processing
  uint8_t recv_buffer1_[RECV_BUFFER_SIZE]{};
  uint8_t recv_buffer2_[RECV_BUFFER_SIZE]{};
  // We start by receiving into recv_buffer1, so processing_buffer is the 2 (but empty)
  uint8_t *volatile processing_buffer_ = recv_buffer2_;
  volatile ssize_t processing_buffer_len_ = 0;

  SerialDriver* slink{};
  SerialDriver* rtklink{};

  thread_t *processing_thread_ = nullptr;
  std::recursive_mutex mutex_;
  // This is reset by the receiving ISR and set by the thread to signal if it's safe to process more data.
  volatile bool processing_done_ = true;
  bool stopped_ = true;

  void threadFunc();

  static void threadHelper(void *instance);
};
}  // namespace xbot::driver::gps

#endif  // XBOT_DRIVER_GPS_GPS_INTERFACE_H
