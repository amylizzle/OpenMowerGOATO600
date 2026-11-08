/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "camera_service.hpp"

#include <ulog.h>

#include <cstdio>
#include <string>
#include <xbot-service/Io.hpp>
#include <xbot-service/portable/system.hpp>

using xbot::driver::camera::CameraDriver;
using xbot::driver::camera::FrameInfo;
using xbot::driver::camera::FrameReader;

bool CameraService::OnStart() {
  if (driver_ == nullptr) {
    // We don't have a driver running yet, so create one.
    driver_ = std::make_unique<CameraDriver>();
  }

  want_stream_ = true;
  std::string reply;
  if (driver_->EnableStream(reply)) {
    ULOG_INFO("Camera: enabled stream: %s", reply.c_str());
  } else {
    // Not fatal: the daemon may still be coming up, and tick() keeps asking.
    ULOG_WARNING("Camera: could not enable stream: %s", reply.c_str());
  }

  preview_url_ = PreviewUrl();

  if (web_server_ == nullptr) {
    web_server_ = std::make_unique<goat::camera::CameraWebServer>(*driver_, kPreviewDecimation);
  }
  if (web_server_->Start(kPreviewPort)) {
    ULOG_INFO("Camera: preview at %s", preview_url_.c_str());
  } else {
    // A preview we cannot serve is no reason to hold up the rest of the robot.
    ULOG_WARNING("Camera: preview server did not start; carrying on without it");
  }

  last_progress_micros_ = xbot::service::system::getTimeMicros();
  last_sample_micros_ = last_progress_micros_;
  have_sequence_ = false;
  return true;
}

void CameraService::OnStop() {
  want_stream_ = false;
  if (web_server_ != nullptr) {
    web_server_->Stop();
  }
  reader_.reset();
  if (driver_ != nullptr) {
    std::string reply;
    driver_->DisableStream(reply);
  }
  streaming_ = false;
}

void CameraService::OnStreamEnableChanged(const uint8_t& new_value) {
  if (driver_ == nullptr) {
    return;
  }
  std::string reply;
  want_stream_ = new_value != 0;
  if (want_stream_) {
    if (driver_->EnableStream(reply)) {
      ULOG_INFO("Camera: stream enabled on request: %s", reply.c_str());
    } else {
      ULOG_WARNING("Camera: could not enable stream on request: %s", reply.c_str());
    }
    last_progress_micros_ = xbot::service::system::getTimeMicros();
  } else {
    driver_->DisableStream(reply);
    ULOG_INFO("Camera: stream disabled on request: %s", reply.c_str());
    streaming_ = false;
  }
}

std::string CameraService::PreviewUrl() const {
  char address[16]{};
  uint16_t port = 0;
  if (!xbot::service::Io::getEndpoint(address, sizeof(address), &port)) {
    // We still know the port we serve on, just not which address to reach it at.
    return "http://<mower>:" + std::to_string(kPreviewPort) + "/";
  }
  return std::string("http://") + address + ":" + std::to_string(kPreviewPort) + "/";
}

bool CameraService::EnsureReader() {
  if (reader_ != nullptr) {
    return true;
  }
  reader_ = driver_->OpenReader();
  return reader_ != nullptr;
}

void CameraService::RecoverStream() {
  // Either the daemon was restarted or it dropped our client, so ask again and
  // start from a fresh mapping: the old one may belong to a dead segment.
  std::string reply;
  driver_->EnableStream(reply);
  reader_.reset();
  have_sequence_ = false;
  ULOG_WARNING("Camera: no frames for %us, re-enabling the stream: %s", kStreamStallMicros / 1'000'000,
               reply.c_str());
}

void CameraService::tick() {
  if (driver_ == nullptr) {
    return;
  }

  const uint32_t now = xbot::service::system::getTimeMicros();
  const uint32_t since_sample = now - last_sample_micros_;
  last_sample_micros_ = now;

  FrameInfo info{};
  const bool have_frame = EnsureReader() && reader_->Peek(info);

  if (have_frame) {
    width_ = info.width;
    height_ = info.height;

    if (!have_sequence_) {
      // First frame after a (re)start: nothing to measure a rate against yet.
      have_sequence_ = true;
      last_sequence_ = info.sequence;
      last_progress_micros_ = now;
      frame_rate_ = 0.0f;
    } else {
      const int32_t published = info.sequence - last_sequence_;
      last_sequence_ = info.sequence;
      if (published > 0) {
        frame_count_ += static_cast<uint32_t>(published);
        last_progress_micros_ = now;
        frame_rate_ = since_sample > 0 ? static_cast<float>(published) * 1'000'000.0f / static_cast<float>(since_sample)
                                       : 0.0f;
      } else {
        // The counter stood still (or the daemon restarted and reset it), so
        // we are not getting frames right now whatever the header says.
        frame_rate_ = 0.0f;
      }
    }
  } else {
    frame_rate_ = 0.0f;
  }

  const bool stalled = now - last_progress_micros_ > kStreamStallMicros;
  const bool streaming = have_frame && !stalled;
  if (streaming != streaming_) {
    streaming_ = streaming;
    if (streaming) {
      ULOG_INFO("Camera: streaming %ux%u", width_, height_);
    } else {
      ULOG_WARNING("Camera: stream stopped. Is cameraProvider.service running?");
    }
  }

  // Only chase the stream if we are supposed to be holding it; a deliberate
  // StreamEnable=0 should stay off.
  if (stalled && want_stream_) {
    RecoverStream();
    last_progress_micros_ = now;
  }

  StartTransaction();
  SendStreamActive(streaming_ ? 1 : 0);
  const uint16_t size[2] = {static_cast<uint16_t>(width_), static_cast<uint16_t>(height_)};
  SendFrameSize(size, 2);
  SendFrameRate(frame_rate_);
  SendFrameCount(frame_count_);
  SendPreviewURL(preview_url_.c_str(), preview_url_.length());
  CommitTransaction();
}
