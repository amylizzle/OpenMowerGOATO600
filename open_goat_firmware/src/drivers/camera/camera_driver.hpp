/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Driver for the GOAT O600 fisheye camera.
 *
 * The camera is owned by the stock EcoVacs `cameraProvider` daemon; there is no
 * V4L2 device node for it. Streaming is enabled over a UNIX control socket and
 * frames appear in a double-buffered POSIX shared memory segment as NV12. Both
 * the protocol and the buffer layout are reverse engineered - see
 * hardware_control_RE/camtest.py, which this driver is a port of.
 *
 * Note that the project README tells you to disable cameraProvider.service as
 * part of switching off the EcoVacs stack. It has to be running for this
 * driver to produce anything.
 *
 * This header deliberately depends on nothing but libstdc++ and POSIX: it is
 * compiled both into the firmware (by camera_service) and into the ROS node in
 * src/goat_ros, so the one description we have of this format lives in one
 * place.
 */

#ifndef CAMERA_DRIVER_HPP
#define CAMERA_DRIVER_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace xbot::driver::camera {

// Defaults for the stock EcoVacs daemon on a GOAT O600.
inline constexpr const char* kDefaultShmPath = "/dev/shm/shm_fisheye2";
inline constexpr const char* kDefaultControlSocket = "/hosttmp/camerasockt.domain";

/**
 * Geometry and identity of one frame in the shared memory segment.
 */
struct FrameInfo {
  uint32_t width = 0;
  uint32_t height = 0;
  // Frame timestamp straight out of the frame header. Its epoch and latency are
  // uncharacterised, so treat it as an identity for the frame, not as a clock.
  int64_t timestamp = 0;
  // The writer's publish counter, shared by both buffers. Increases by one per
  // published frame, so differences give an exact frame rate and drop count.
  int32_t sequence = 0;
};

/**
 * Reader for the double-buffered NV12 shared memory segment.
 *
 * One reader per consumer: each keeps its own cursor over the segment, so two
 * consumers never race over which frame was "last seen". A reader is not
 * thread safe; give each thread its own.
 */
class FrameReader {
 public:
  FrameReader() = default;
  ~FrameReader();

  FrameReader(const FrameReader&) = delete;
  FrameReader& operator=(const FrameReader&) = delete;

  bool Open(const std::string& shm_path);
  void Close();

  bool IsOpen() const {
    return map_ != nullptr;
  }

  /**
   * Reads the newest frame's header without copying the payload and without
   * advancing the cursor, so a caller that only wants statistics costs nothing.
   * @return true if the header describes a plausible frame.
   */
  bool Peek(FrameInfo& info) const;

  /**
   * Copies out the newest frame this reader has not seen yet, full resolution,
   * as NV12: width*height luma followed by width*height/2 interleaved chroma.
   * @return true if `nv12` now holds a complete frame.
   */
  bool GrabNV12(std::vector<uint8_t>& nv12, FrameInfo& info);

  /**
   * As GrabNV12, but takes every `step`'th row and column and de-interleaves
   * the chroma, producing planar YUV420 (I420) at the decimated size. Dropping
   * samples is cheap - no filtering, no per-pixel arithmetic - which is what
   * the browser preview wants.
   * @param step decimation factor, 1 for full resolution.
   * @param out_width  decimated width, always even.
   * @param out_height decimated height, always even.
   * @return true if `i420` now holds a complete frame.
   */
  bool GrabI420(std::vector<uint8_t>& i420, uint32_t step, uint32_t& out_width, uint32_t& out_height, FrameInfo& info);

 private:
  /**
   * Validates the control block and frame header of whichever buffer the writer
   * published last.
   * @return pointer to that frame's header, or nullptr if it is unusable.
   */
  const uint8_t* ReadNewestHeader(FrameInfo& info) const;

  /**
   * The writer always fills the *other* buffer, so one publish during our copy
   * is harmless; two means ours was recycled and the copy may be torn.
   */
  bool CopyWasClean(int32_t sequence_before) const;

  uint8_t* map_ = nullptr;
  size_t map_size_ = 0;
  size_t buffer_size_ = 0;
  int64_t last_timestamp_ = 0;
};

/**
 * Where to find the camera. The defaults are the stock daemon's.
 */
struct Config {
  std::string shm_path = kDefaultShmPath;
  std::string control_socket = kDefaultControlSocket;
  // Identifies us to the daemon. It tracks stream clients by name, so this has
  // to be stable across our own stream_on/stream_off calls.
  std::string client_name = "openmower";
};

/**
 * Owns the camera: turns the vendor daemon's stream on and off, and hands out
 * readers for the shared memory it publishes into.
 */
class CameraDriver {
 public:
  explicit CameraDriver(Config config = Config{});

  /** Drops the stream again if we are the ones who asked for it. */
  ~CameraDriver();

  CameraDriver(const CameraDriver&) = delete;
  CameraDriver& operator=(const CameraDriver&) = delete;

  /**
   * Asks cameraProvider to start streaming. This is what the vendor's `sitctl`
   * does. Safe to call again to recover a stream the daemon has dropped.
   * @param reply the daemon's response, or the reason we could not ask it.
   */
  bool EnableStream(std::string& reply);

  /** Asks cameraProvider to stop streaming on our behalf. */
  bool DisableStream(std::string& reply);

  /**
   * True once EnableStream() has succeeded and DisableStream() has not. Safe to
   * read from another thread, so a status page can ask without synchronising
   * with whoever drives the stream.
   */
  bool StreamEnabled() const {
    return stream_enabled_;
  }

  const Config& GetConfig() const {
    return config_;
  }

  /** @return a reader for this camera, or nullptr if the segment is not there. */
  std::unique_ptr<FrameReader> OpenReader() const;

 private:
  Config config_;
  std::atomic<bool> stream_enabled_{false};
};

/**
 * Sends a single stream_on/stream_off command to cameraProvider's control
 * socket. Exposed for callers that want to drive the daemon without owning a
 * CameraDriver.
 * @param reply the daemon's response, or the reason we could not ask it.
 */
bool SendStreamCommand(const std::string& socket_path, const std::string& client_name, bool stream_on,
                       std::string& reply);

}  // namespace xbot::driver::camera

#endif  // CAMERA_DRIVER_HPP
