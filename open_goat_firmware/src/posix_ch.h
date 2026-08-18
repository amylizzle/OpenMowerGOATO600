// POSIX compatibility for ChibiOS-like APIs used in this project.
// This header provides a minimal set of macros/types/functions so existing
// firmware code referencing ch* and HAL UART APIs can compile on Linux.
#ifndef POSIX_CH_H
#define POSIX_CH_H

#include <pthread.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>


// Basic constants
#define MSG_OK 0
#define UART_ERR_NOT_ACTIVE (-1)
#define TIME_INFINITE ((uint32_t)0xFFFFFFFFu)
#define ALL_EVENTS 0xFFFFFFFFu
#define TIME_MS2I(x) (x)

// System time helpers (milliseconds since steady clock)
#include <chrono>
inline uint32_t chVTGetSystemTimeX() {
  using namespace std::chrono;
  auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  return static_cast<uint32_t>(ms & 0xFFFFFFFFu);
}

#define TIME_I2S(x) ((x) / 1000u)

// Minimal UART types used in project
struct UARTConfig {
  uint32_t speed{0};
  void *context{nullptr};
  void (*rxend_cb)(struct UARTDriver *uartp){nullptr};
};

struct DmaStreamEmu { std::atomic<size_t> NDTR{0}; };
struct DmaEmu { DmaStreamEmu *stream{nullptr}; };

struct UARTDriver {
  UARTConfig config;
  int fd{-1};
  DmaEmu *dmarx{nullptr};
  std::atomic<bool> rx_active{false};
  std::atomic<size_t> rx_bytes_received{0};
  std::thread rx_thread;
};

// Map common integer baud rates to termios speed_t values.
inline speed_t baud_to_speed(uint32_t baud) {
  switch (baud) {
    case 50: return B50;
    case 75: return B75;
    case 110: return B110;
    case 134: return B134;
    case 150: return B150;
    case 200: return B200;
    case 300: return B300;
    case 600: return B600;
    case 1200: return B1200;
    case 1800: return B1800;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
#ifdef B57600
    case 57600: return B57600;
#endif
#ifdef B115200
    case 115200: return B115200;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return (speed_t)0;
  }
}

// Open and configure a serial device into the provided UARTDriver.
// Returns 0 on success, -1 on failure.
inline int uartOpenPort(UARTDriver *uart, const char *device, uint32_t baud) {
  if (!uart || !device) return -1;
  uart->config.speed = baud;
  int fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    return -1;
  }

  cfmakeraw(&tty);
  speed_t sp = baud_to_speed(baud);
  if (sp != (speed_t)0) {
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);
  }
  // Non-blocking reads with minimum characters 0 and no inter-character timer
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    return -1;
  }

  uart->fd = fd;
  return 0;
}

// Close UART port and cleanup emulated DMA/threads
inline void uartClose(UARTDriver *uart) {
  if (!uart) return;
  // stop receiver thread (directly stop and join to avoid forward-decl dependency)
  if (uart->rx_active) {
    uart->rx_active = false;
  }
  if (uart->rx_thread.joinable()) uart->rx_thread.join();
  if (uart->fd >= 0) {
    ::close(uart->fd);
    uart->fd = -1;
  }
  if (uart->dmarx) {
    if (uart->dmarx->stream) delete uart->dmarx->stream;
    delete uart->dmarx;
    uart->dmarx = nullptr;
  }
}

// Convenience factory that allocates and opens a UARTDriver. Returns nullptr on failure.
inline UARTDriver *CreateUARTDriver(const char *device, uint32_t baud) {
  UARTDriver *u = new UARTDriver();
  if (uartOpenPort(u, device, baud) != 0) {
    delete u;
    return nullptr;
  }
  return u;
}

inline void DestroyUARTDriver(UARTDriver *uart) {
  if (!uart) return;
  uartClose(uart);
  delete uart;
}

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

// System lock (cooperative replacement)
inline std::mutex &ch_sys_mutex() { static std::mutex m; return m; }
inline void chSysLock() { ch_sys_mutex().lock(); }
inline void chSysUnlock() { ch_sys_mutex().unlock(); }
inline void chSysLockFromISR() { chSysLock(); }
inline void chSysUnlockFromISR() { chSysUnlock(); }

// Events
inline void chEvtSignalI(thread_t *tp, uint32_t flags) {
  if (!tp) return;
  std::lock_guard<std::mutex> lk(tp->ev_mutex);
  tp->ev_flags |= flags;
  tp->ev_cv.notify_one();
}
inline uint32_t chEvtWaitAnyTimeout(uint32_t mask, uint32_t timeout_ms) {
  static thread_local thread_t *self = nullptr;
  if (!self) {
    self = new thread_t();
    self->id = std::this_thread::get_id();
  }
  std::unique_lock<std::mutex> lk(self->ev_mutex);
  if (self->ev_flags & mask) {
    uint32_t f = self->ev_flags & mask;
    self->ev_flags &= ~mask;
    return f;
  }
  if (timeout_ms == TIME_INFINITE) {
    self->ev_cv.wait(lk, [&]{ return (self->ev_flags & mask) != 0; });
    uint32_t f = self->ev_flags & mask;
    self->ev_flags &= ~mask;
    return f;
  }
  auto status = self->ev_cv.wait_for(lk, std::chrono::milliseconds(timeout_ms));
  if (status == std::cv_status::timeout) {
    return 0;
  }
  uint32_t f = self->ev_flags & mask;
  self->ev_flags &= ~mask;
  return f;
}
inline void chEvtBroadcastFlags(thread_t *tp, uint32_t flags) { chEvtSignalI(tp, flags); }

// UART stubs: minimal non-blocking support
inline int uartStart(UARTDriver *uart, UARTConfig *config) { if(!uart || !config) return -1; uart->config = *config; if(!uart->dmarx) { uart->dmarx = new DmaEmu(); uart->dmarx->stream = new DmaStreamEmu(); } return MSG_OK; }
inline int uartStartReceive(UARTDriver *uart, size_t bufsize, uint8_t *buffer) {
  if (!uart) return -1;
  if (uart->fd < 0) return -1;

  if (uart->rx_thread.joinable()) {
      uart->rx_active = false;      // Signal the thread loop to exit
      uart->rx_thread.join();       // Wait for the thread to actually finish
  }

  uart->rx_active = true;
  uart->rx_bytes_received = 0;

  uart->rx_active = true;
  uart->rx_bytes_received = 0;
  uart->rx_thread = std::thread([uart, bufsize, buffer]() {
    while (uart->rx_active) {
      ssize_t r = ::read(uart->fd, buffer, bufsize);
      if (r > 0) {
        uart->rx_bytes_received = (size_t)r;
        if (uart->dmarx && uart->dmarx->stream) uart->dmarx->stream->NDTR = bufsize - r;
        if (uart->config.rxend_cb) uart->config.rxend_cb(uart);
      } else if (r == 0) {
        uart->rx_active = false;
      } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        uart->rx_active = false;
      }
    }
  });
  return 0;
}
inline int uartStartReceiveI(UARTDriver *uart, size_t bufsize, uint8_t *buffer) { return uartStartReceive(uart, bufsize, buffer); }
inline ssize_t uartStopReceiveI(UARTDriver *uart) { if(!uart) return UART_ERR_NOT_ACTIVE; if(!uart->rx_active) return UART_ERR_NOT_ACTIVE; uart->rx_active = false; if(uart->rx_thread.joinable()) uart->rx_thread.join(); return 0; }
inline bool uartSendFullTimeout(UARTDriver *uart, const size_t *len, const void *data, uint32_t timeout_ms) { if(!uart||uart->fd<0) return false; const uint8_t *p = (const uint8_t*)data; size_t to_write = *len; size_t written=0; auto deadline = std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms==TIME_INFINITE?1000000000:timeout_ms); while(written<to_write){ ssize_t w = ::write(uart->fd, p+written, to_write-written); if(w>0){ written+=(size_t)w; continue; } if(errno==EAGAIN||errno==EWOULDBLOCK){ if(std::chrono::steady_clock::now()>deadline) break; std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; } return false; } return written==to_write; }

#endif // POSIX_CH_H
