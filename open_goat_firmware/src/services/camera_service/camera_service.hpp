/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Owns the GOAT's fisheye camera.
 *
 * The frames themselves do not go over the xbot service bus - a single data
 * packet is capped at one Ethernet frame (xbot::config::max_packet_size) and a
 * raw 1920x1080 NV12 frame is about three megabytes. So this service publishes
 * the camera's *state* (is it streaming, at what size and rate) and takes care
 * of the two things that actually need owning: telling the vendor daemon to
 * stream, and keeping that stream alive.
 *
 * Pixels are read straight out of shared memory by whoever wants them:
 *  - the built-in preview server, for a browser (see camera_web_server.hpp);
 *  - the ROS node in src/goat_ros, which publishes camera/image_raw.
 * The shared memory is read-only and every reader keeps its own cursor, so any
 * number of them can run at once.
 */

#ifndef CAMERA_SERVICE_HPP
#define CAMERA_SERVICE_HPP

#include <cstdint>
#include <memory>

#include <CameraServiceBase.hpp>
#include <drivers/camera/camera_driver.hpp>

#include "camera_web_server.hpp"

using namespace xbot::service;

class CameraService : public CameraServiceBase {
 public:
  // nginx already has 8080 and rosbridge conventionally takes 9090.
  static constexpr uint16_t kPreviewPort = 8081;
  // 1920x1080 decimated by 4 is 480x270: a usable preview for a fraction of
  // the bytes and none of the CPU a rescale would cost.
  static constexpr uint32_t kPreviewDecimation = 4;
  // How long the shared memory may go without a new frame before we assume the
  // daemon dropped us and ask it to stream again.
  static constexpr uint32_t kStreamStallMicros = 5'000'000;

  explicit CameraService(const uint16_t service_id) : CameraServiceBase(service_id) {
  }

  /** @return the most recent frame size we saw, or 0x0 if there is no stream. */
  uint32_t GetFrameWidth() const {
    return width_;
  }
  uint32_t GetFrameHeight() const {
    return height_;
  }
  bool IsStreaming() const {
    return streaming_;
  }

 protected:
  bool OnStart() override;
  void OnStop() override;
  void OnStreamEnableChanged(const uint8_t& new_value) override;

 private:
  void tick();

  /** Opens the shared memory if it is not open yet. */
  bool EnsureReader();

  /** Re-asks the daemon to stream and drops our mapping so we pick up a new one. */
  void RecoverStream();

  /** Builds "http://<our address>:<preview port>/" for the PreviewURL output. */
  std::string PreviewUrl() const;

  std::unique_ptr<xbot::driver::camera::CameraDriver> driver_;
  std::unique_ptr<goat::camera::CameraWebServer> web_server_;
  // The service's own reader, used only to read frame headers for the
  // statistics below. The preview server opens its own.
  std::unique_ptr<xbot::driver::camera::FrameReader> reader_;

  std::string preview_url_;

  // Whether we are supposed to be holding the stream. Not the same as the
  // driver's StreamEnabled(): if the daemon was not reachable at startup we
  // still want the stream, and the watchdog has to keep asking for it.
  bool want_stream_ = true;

  uint32_t width_ = 0;
  uint32_t height_ = 0;
  float frame_rate_ = 0.0f;
  // Frames published since this service started, accumulated across daemon
  // restarts (which reset the counter in shared memory).
  uint32_t frame_count_ = 0;
  bool streaming_ = false;

  // Last publish counter we saw, and when we saw it change.
  int32_t last_sequence_ = 0;
  bool have_sequence_ = false;
  uint32_t last_progress_micros_ = 0;
  uint32_t last_sample_micros_ = 0;

  // Published once a second: fast enough to notice the camera dying, slow
  // enough to be free.
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 1'000'000,
                                 XBOT_FUNCTION_FOR_METHOD(CameraService, &CameraService::tick, this)};
};

#endif  // CAMERA_SERVICE_HPP
