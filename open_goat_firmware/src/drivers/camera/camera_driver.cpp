/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "camera_driver.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace xbot::driver::camera {
namespace {

// Shared memory layout. Offsets are from camtest.py and are the only
// description of this format we have - the daemon itself is a vendor binary.
constexpr size_t kCtrlSize = 0xB0;     // mutex, buffer index, counter, 2 rwlocks
constexpr size_t kHdrSize = 0x30;      // per-frame header
constexpr size_t kIndexOffset = 0x30;  // int32 buffer index, then int32 counter
constexpr size_t kTsOffset = 0x08;     // int64 frame timestamp
constexpr size_t kSizeOffset = 0x20;   // uint32 payload size, then width, height

// Sanity bound on the frame dimensions read out of the header, so a garbage
// header can never turn into a huge read. The sensor is 1920x1080.
constexpr uint32_t kMaxDimension = 8192;

constexpr int kControlTimeoutSeconds = 3;

template <typename T>
T ReadAt(const uint8_t* base, size_t offset) {
  T value{};
  std::memcpy(&value, base + offset, sizeof(value));
  return value;
}

}  // namespace

bool SendStreamCommand(const std::string& socket_path, const std::string& client_name, bool stream_on,
                       std::string& reply) {
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (socket_path.size() >= sizeof(addr.sun_path)) {
    reply = "control socket path too long";
    return false;
  }
  std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    reply = std::string("socket() failed: ") + std::strerror(errno);
    return false;
  }

  timeval timeout{};
  timeout.tv_sec = kControlTimeoutSeconds;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    reply = std::string("connect() failed: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }

  const std::string message = "camera0 {\"stream_on\":" + std::string(stream_on ? "1" : "0") + ",\"name\":\"" +
                              client_name + "\",\"type\":\"socket_default\"}";
  if (write(fd, message.data(), message.size()) != static_cast<ssize_t>(message.size())) {
    reply = std::string("write() failed: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }

  char buffer[4096];
  const ssize_t received = read(fd, buffer, sizeof(buffer) - 1);
  if (received > 0) {
    buffer[received] = '\0';
    reply = buffer;
  } else {
    reply.clear();
  }
  ::close(fd);
  return true;
}

FrameReader::~FrameReader() {
  Close();
}

bool FrameReader::Open(const std::string& shm_path) {
  Close();

  const int fd = ::open(shm_path.c_str(), O_RDONLY);
  if (fd < 0) {
    return false;
  }
  struct stat st{};
  if (fstat(fd, &st) != 0 || st.st_size < 0 ||
      static_cast<size_t>(st.st_size) <= kCtrlSize + 2 * kHdrSize) {
    ::close(fd);
    return false;
  }

  void* map = mmap(nullptr, static_cast<size_t>(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);  // the mapping keeps its own reference
  if (map == MAP_FAILED) {
    return false;
  }

  map_ = static_cast<uint8_t*>(map);
  map_size_ = static_cast<size_t>(st.st_size);
  buffer_size_ = (map_size_ - kCtrlSize) / 2;
  last_timestamp_ = 0;
  return true;
}

void FrameReader::Close() {
  if (map_ != nullptr) {
    munmap(map_, map_size_);
    map_ = nullptr;
    map_size_ = 0;
    buffer_size_ = 0;
  }
}

const uint8_t* FrameReader::ReadNewestHeader(FrameInfo& info) const {
  if (map_ == nullptr) {
    return nullptr;
  }

  const int32_t index = ReadAt<int32_t>(map_, kIndexOffset);
  const int32_t sequence = ReadAt<int32_t>(map_, kIndexOffset + sizeof(int32_t));
  if (index < 0 || index > 1) {
    return nullptr;
  }

  const uint8_t* const base = map_ + kCtrlSize + static_cast<size_t>(index) * buffer_size_;
  const int64_t timestamp = ReadAt<int64_t>(base, kTsOffset);
  const uint32_t payload_size = ReadAt<uint32_t>(base, kSizeOffset);
  const uint32_t width = ReadAt<uint32_t>(base, kSizeOffset + 4);
  const uint32_t height = ReadAt<uint32_t>(base, kSizeOffset + 8);

  if (timestamp == 0 || payload_size == 0 || payload_size > buffer_size_ - kHdrSize) {
    return nullptr;
  }
  // NV12 needs even dimensions for the half-resolution chroma plane.
  if (width == 0 || height == 0 || width > kMaxDimension || height > kMaxDimension || (width % 2) != 0 ||
      (height % 2) != 0) {
    return nullptr;
  }
  if (static_cast<size_t>(width) * height * 3 / 2 > buffer_size_ - kHdrSize) {
    return nullptr;
  }

  info.width = width;
  info.height = height;
  info.timestamp = timestamp;
  info.sequence = sequence;
  return base;
}

bool FrameReader::CopyWasClean(int32_t sequence_before) const {
  const int32_t sequence_after = ReadAt<int32_t>(map_, kIndexOffset + sizeof(int32_t));
  return sequence_after - sequence_before <= 1;
}

bool FrameReader::Peek(FrameInfo& info) const {
  return ReadNewestHeader(info) != nullptr;
}

bool FrameReader::GrabNV12(std::vector<uint8_t>& nv12, FrameInfo& info) {
  const uint8_t* const base = ReadNewestHeader(info);
  if (base == nullptr || info.timestamp == last_timestamp_) {
    return false;  // nothing new since last time
  }

  const size_t frame_size = static_cast<size_t>(info.width) * info.height * 3 / 2;
  nv12.resize(frame_size);
  std::memcpy(nv12.data(), base + kHdrSize, frame_size);

  if (!CopyWasClean(info.sequence)) {
    return false;
  }

  last_timestamp_ = info.timestamp;
  return true;
}

bool FrameReader::GrabI420(std::vector<uint8_t>& i420, uint32_t step, uint32_t& out_width, uint32_t& out_height,
                           FrameInfo& info) {
  if (step == 0) {
    step = 1;
  }

  const uint8_t* const base = ReadNewestHeader(info);
  if (base == nullptr || info.timestamp == last_timestamp_) {
    return false;  // nothing new since last time
  }

  // Round down to even so the decimated frame still has a whole chroma plane.
  // Both planes are then sampled strictly inside the frame: the last column we
  // touch is at most width-step, and the last row at most height-step.
  const uint32_t width = (info.width / step) & ~1u;
  const uint32_t height = (info.height / step) & ~1u;
  if (width == 0 || height == 0) {
    return false;
  }

  const uint8_t* const luma = base + kHdrSize;
  const uint8_t* const chroma = luma + static_cast<size_t>(info.width) * info.height;

  i420.resize(static_cast<size_t>(width) * height * 3 / 2);
  uint8_t* out = i420.data();

  for (uint32_t row = 0; row < height; ++row) {
    const uint8_t* src = luma + static_cast<size_t>(row) * step * info.width;
    for (uint32_t col = 0; col < width; ++col) {
      *out++ = src[static_cast<size_t>(col) * step];
    }
  }
  // NV12 stores chroma as interleaved U,V pairs; I420 wants the two planes
  // separately, so walk the same rows twice at a stride of two samples.
  for (uint32_t plane = 0; plane < 2; ++plane) {
    for (uint32_t row = 0; row < height / 2; ++row) {
      const uint8_t* src = chroma + static_cast<size_t>(row) * step * info.width + plane;
      for (uint32_t col = 0; col < width / 2; ++col) {
        *out++ = src[static_cast<size_t>(col) * 2 * step];
      }
    }
  }

  if (!CopyWasClean(info.sequence)) {
    return false;
  }

  last_timestamp_ = info.timestamp;
  out_width = width;
  out_height = height;
  return true;
}

CameraDriver::CameraDriver(Config config) : config_(std::move(config)) {
}

CameraDriver::~CameraDriver() {
  if (stream_enabled_) {
    std::string reply;
    DisableStream(reply);
  }
}

bool CameraDriver::EnableStream(std::string& reply) {
  if (!SendStreamCommand(config_.control_socket, config_.client_name, true, reply)) {
    return false;
  }
  stream_enabled_ = true;
  return true;
}

bool CameraDriver::DisableStream(std::string& reply) {
  const bool ok = SendStreamCommand(config_.control_socket, config_.client_name, false, reply);
  // Either the daemon dropped us or it is not reachable at all; in both cases
  // we no longer hold the stream.
  stream_enabled_ = false;
  return ok;
}

std::unique_ptr<FrameReader> CameraDriver::OpenReader() const {
  auto reader = std::make_unique<FrameReader>();
  if (!reader->Open(config_.shm_path)) {
    return nullptr;
  }
  return reader;
}

}  // namespace xbot::driver::camera
