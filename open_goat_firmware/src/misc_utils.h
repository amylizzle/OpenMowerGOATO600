// POSIX compatibility for ChibiOS-like APIs used in this project.
// This header provides a minimal set of macros/types/functions so existing
// firmware code referencing ch* and HAL UART APIs can compile on Linux.
#ifndef MISC_UTILS_H
#define MISC_UTILS_H

#include <pthread.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstddef>
#include <cassert>
#include <etl/algorithm.h>

// Little-endian packet readers shared across firmware drivers.
inline int16_t ReadI16Le(const uint8_t* data, size_t offset) {
  if (!data) return 0;
  return static_cast<int16_t>(static_cast<uint16_t>(data[offset]) |
                               (static_cast<uint16_t>(data[offset + 1]) << 8));
}

inline int32_t ReadI32Le(const uint8_t* data, size_t offset) {
  if (!data) return 0;
  return static_cast<int32_t>(static_cast<uint32_t>(data[offset]) |
                               (static_cast<uint32_t>(data[offset + 1]) << 8) |
                               (static_cast<uint32_t>(data[offset + 2]) << 16) |
                               (static_cast<uint32_t>(data[offset + 3]) << 24));
}

inline uint16_t ReadU16Le(const uint8_t* data, size_t offset) {
  if (!data) return 0;
  return static_cast<uint16_t>(static_cast<uint16_t>(data[offset]) |
                                (static_cast<uint16_t>(data[offset + 1]) << 8));
}

inline uint32_t ReadU32Le(const uint8_t* data, size_t offset) {
  if (!data) return 0;
  return static_cast<uint32_t>(static_cast<uint32_t>(data[offset]) |
                                (static_cast<uint32_t>(data[offset + 1]) << 8) |
                                (static_cast<uint32_t>(data[offset + 2]) << 16) |
                                (static_cast<uint32_t>(data[offset + 3]) << 24));
}

// System time helpers (milliseconds since steady clock)
#include <chrono>
inline uint32_t chVTGetSystemTimeX() {
  using namespace std::chrono;
  auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  return static_cast<uint32_t>(ms & 0xFFFFFFFFu);
}

#define TIME_I2S(x) ((x) / 1000u)


// Thread type
struct thread_t {
  std::thread::id id{};
  std::thread native_thread;
  char name[32]{};
  std::mutex ev_mutex;
  std::condition_variable ev_cv;
  uint32_t ev_flags{0};
};

// Debug/assert
#define DbgAssert(expr, msg) { assert(expr && msg); }

// Thread create
inline thread_t *createThread(void (*fn)(void *), void *arg) {
  thread_t *t = new thread_t();
  t->native_thread = std::thread([t,fn,arg]() { t->id = std::this_thread::get_id(); fn(arg); });
  return t;
}


inline bool TimeoutReached(uint32_t duration, uint32_t delay, uint32_t& block_time) {
  if (duration >= delay) {
    return true;
  } else {
    block_time = etl::min(block_time, delay - duration);
    return false;
  }
}

#endif // MISC_UTILS_H
