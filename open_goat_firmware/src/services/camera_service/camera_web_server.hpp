/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A deliberately small HTTP server that serves a live preview of the camera,
 * so you can point a browser at the mower and see what it sees. It is a port
 * of the preview half of hardware_control_RE/camtest.py.
 *
 * There is no JPEG encoder in the firmware and no point adding one for a
 * preview, so frames go to the browser as decimated planar YUV420 and a few
 * lines of JavaScript convert them on a canvas. That keeps the cost on this
 * side to a strided copy out of shared memory.
 *
 * This is a viewer, not an API: anything that wants the frames for real should
 * take the ROS topic published by src/goat_ros instead.
 */

#ifndef CAMERA_WEB_SERVER_HPP
#define CAMERA_WEB_SERVER_HPP

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <drivers/camera/camera_driver.hpp>

namespace goat::camera {

class CameraWebServer {
 public:
  /**
   * @param driver the camera to stream from. Must outlive the server.
   * @param preview_step decimation applied before sending to the browser.
   *        4 turns the sensor's 1920x1080 into 480x270, which is plenty for a
   *        preview and about a sixteenth of the bytes.
   */
  CameraWebServer(const xbot::driver::camera::CameraDriver& driver, uint32_t preview_step);
  ~CameraWebServer();

  CameraWebServer(const CameraWebServer&) = delete;
  CameraWebServer& operator=(const CameraWebServer&) = delete;

  /**
   * Binds the port and starts serving in the background.
   * @return false if the port could not be bound, in which case nothing is
   *         running and the service can carry on without a preview.
   */
  bool Start(uint16_t port);

  /** Stops serving and waits for the in-flight connections to notice. */
  void Stop();

  bool IsRunning() const {
    return running_;
  }

 private:
  void AcceptLoop();
  void ServeConnection(int client_fd);
  void ServeStream(int client_fd);

  const xbot::driver::camera::CameraDriver& driver_;
  const uint32_t preview_step_;

  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  // Connections are served on detached threads, so Stop() waits on this
  // instead of joining them.
  std::atomic<int> client_count_{0};
  std::thread accept_thread_;
};

}  // namespace goat::camera

#endif  // CAMERA_WEB_SERVER_HPP
