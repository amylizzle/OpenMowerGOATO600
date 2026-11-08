/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "camera_web_server.hpp"

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ulog.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <vector>

namespace goat::camera {
namespace {

// Enough for a handful of browser tabs; each one costs a thread and a strided
// copy per frame, and this runs on the mower's CPU alongside everything else.
constexpr int kMaxClients = 4;
constexpr int kAcceptTimeoutSeconds = 1;
constexpr int kRequestTimeoutSeconds = 5;
constexpr size_t kMaxRequestSize = 4096;
// The source runs at about 10 fps; poll well under that so we pick frames up
// promptly without busy-waiting.
constexpr auto kPollInterval = std::chrono::milliseconds(5);

constexpr char kPage[] =
    R"HTML(<!doctype html>
<meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>GOAT camera</title>
<style>
  html,body{margin:0;height:100%;background:#111;color:#eee;
            font:13px/1.4 ui-monospace,SFMono-Regular,Menlo,monospace}
  #c{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}
  #s{position:absolute;left:0;right:0;bottom:0;padding:6px 10px;
     background:rgba(0,0,0,.55);backdrop-filter:blur(4px)}
</style>
<canvas id=c></canvas>
<div id=s>connecting...</div>
<script>
const c = document.getElementById('c'), g = c.getContext('2d'), s = document.getElementById('s');
let img = null, frames = 0, bytes = 0, since = performance.now();

// YUV420 -> RGBA on the canvas. BT.601 with the usual studio-swing offsets,
// which is what the sensor pipeline hands us.
function draw(w, h, d) {
  if (!img || c.width != w || c.height != h) {
    c.width = w; c.height = h; img = g.createImageData(w, h);
  }
  const p = img.data, cw = w >> 1, us = w * h, vs = us + cw * (h >> 1);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const Y = 1.164 * (d[y * w + x] - 16), i = (y >> 1) * cw + (x >> 1),
            U = d[us + i] - 128, V = d[vs + i] - 128, o = (y * w + x) << 2;
      p[o] = Y + 1.596 * V;
      p[o + 1] = Y - 0.813 * V - 0.391 * U;
      p[o + 2] = Y + 2.018 * U;
      p[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
}

function stats(w, h) {
  const now = performance.now(), dt = (now - since) / 1000;
  if (dt < 1) return;
  s.textContent = w + 'x' + h + '  ' + (frames / dt).toFixed(1) + ' fps  ' +
                  (bytes / dt / 1024).toFixed(0) + ' KiB/s';
  frames = 0; bytes = 0; since = now;
}

// Frames arrive back to back on one response: a 12 byte big-endian header of
// width, height and payload length, then that many bytes of YUV420.
function connect() {
  fetch('stream').then(r => {
    const rd = r.body.getReader();
    let buf = new Uint8Array(0);
    const pump = ({done, value}) => {
      if (done) throw new Error('stream ended');
      const n = new Uint8Array(buf.length + value.length);
      n.set(buf); n.set(value, buf.length); buf = n;
      bytes += value.length;
      for (;;) {
        if (buf.length < 12) break;
        const dv = new DataView(buf.buffer, buf.byteOffset, 12),
              w = dv.getUint32(0), h = dv.getUint32(4), len = dv.getUint32(8);
        if (buf.length < 12 + len) break;
        draw(w, h, buf.subarray(12, 12 + len));
        buf = buf.slice(12 + len);
        frames++;
        stats(w, h);
      }
      return rd.read().then(pump);
    };
    return rd.read().then(pump);
  }).catch(e => {
    s.textContent = 'disconnected (' + e.message + '), retrying...';
    setTimeout(connect, 1000);
  });
}
connect();
</script>
)HTML";

bool SendAll(int fd, const void* data, size_t length) {
  const auto* cursor = static_cast<const uint8_t*>(data);
  while (length > 0) {
    // MSG_NOSIGNAL: a browser tab closing mid-frame must not take the firmware
    // down with a SIGPIPE.
    const ssize_t sent = send(fd, cursor, length, MSG_NOSIGNAL);
    if (sent <= 0) {
      if (sent < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    cursor += sent;
    length -= static_cast<size_t>(sent);
  }
  return true;
}

bool SendAll(int fd, const std::string& data) {
  return SendAll(fd, data.data(), data.size());
}

bool SendSimpleResponse(int fd, const char* status, const char* content_type, const std::string& body) {
  const std::string response = std::string("HTTP/1.1 ") + status + "\r\n" +
                               "Content-Type: " + content_type + "\r\n" +
                               "Content-Length: " + std::to_string(body.size()) + "\r\n" +
                               "Cache-Control: no-store\r\n"
                               "Connection: close\r\n\r\n" +
                               body;
  return SendAll(fd, response);
}

/**
 * Reads up to the end of the request head and returns the requested path.
 * This understands just enough HTTP to serve a preview page: one GET, no
 * keep-alive, no headers we care about.
 * @return the path, or an empty string if this is not a request we will serve.
 */
std::string ReadRequestPath(int fd) {
  std::string request;
  char chunk[1024];
  while (request.find("\r\n\r\n") == std::string::npos && request.size() < kMaxRequestSize) {
    const ssize_t received = recv(fd, chunk, sizeof(chunk), 0);
    if (received <= 0) {
      if (received < 0 && errno == EINTR) {
        continue;
      }
      return {};
    }
    request.append(chunk, static_cast<size_t>(received));
  }

  if (request.compare(0, 4, "GET ") != 0) {
    return {};
  }
  const size_t path_start = 4;
  const size_t path_end = request.find(' ', path_start);
  if (path_end == std::string::npos) {
    return {};
  }
  std::string path = request.substr(path_start, path_end - path_start);
  // Query strings and fragments make no difference to any of our routes.
  const size_t cut = path.find_first_of("?#");
  if (cut != std::string::npos) {
    path.resize(cut);
  }
  return path.empty() ? std::string("/") : path;
}

void SetTimeout(int fd, int option, int seconds) {
  timeval timeout{};
  timeout.tv_sec = seconds;
  setsockopt(fd, SOL_SOCKET, option, &timeout, sizeof(timeout));
}

}  // namespace

CameraWebServer::CameraWebServer(const xbot::driver::camera::CameraDriver& driver, uint32_t preview_step)
    : driver_(driver), preview_step_(preview_step == 0 ? 1 : preview_step) {
}

CameraWebServer::~CameraWebServer() {
  Stop();
}

bool CameraWebServer::Start(uint16_t port) {
  if (running_) {
    return true;
  }

  listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    ULOG_ERROR("Camera preview: socket() failed: %s", std::strerror(errno));
    return false;
  }

  const int reuse = 1;
  setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  // So the accept loop wakes up often enough to notice Stop().
  SetTimeout(listen_fd_, SO_RCVTIMEO, kAcceptTimeoutSeconds);

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port);
  if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
      listen(listen_fd_, kMaxClients) != 0) {
    ULOG_ERROR("Camera preview: could not listen on port %u: %s", port, std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }

  running_ = true;
  accept_thread_ = std::thread([this] { AcceptLoop(); });
  return true;
}

void CameraWebServer::Stop() {
  if (!running_) {
    return;
  }
  running_ = false;

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }

  // The streaming loops check running_ once per poll interval, so this is a
  // short wait; bound it anyway rather than hanging shutdown on a wedged
  // socket.
  for (int attempt = 0; attempt < 200 && client_count_ > 0; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void CameraWebServer::AcceptLoop() {
  while (running_) {
    const int client_fd = accept(listen_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      // The listening socket has a receive timeout, so this is the normal way
      // out of accept() when nobody is connecting.
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      ULOG_ERROR("Camera preview: accept() failed: %s", std::strerror(errno));
      break;
    }

    if (client_count_ >= kMaxClients) {
      SendSimpleResponse(client_fd, "503 Service Unavailable", "text/plain", "too many viewers\n");
      ::close(client_fd);
      continue;
    }

    ++client_count_;
    std::thread([this, client_fd] {
      ServeConnection(client_fd);
      ::close(client_fd);
      --client_count_;
    }).detach();
  }
}

void CameraWebServer::ServeConnection(int client_fd) {
  SetTimeout(client_fd, SO_RCVTIMEO, kRequestTimeoutSeconds);
  SetTimeout(client_fd, SO_SNDTIMEO, kRequestTimeoutSeconds);

  const std::string path = ReadRequestPath(client_fd);
  if (path.empty()) {
    SendSimpleResponse(client_fd, "400 Bad Request", "text/plain", "expected a GET\n");
    return;
  }

  if (path == "/stream") {
    ServeStream(client_fd);
    return;
  }
  if (path == "/status") {
    const auto& config = driver_.GetConfig();
    xbot::driver::camera::FrameInfo info{};
    const auto reader = driver_.OpenReader();
    const bool have_frame = reader != nullptr && reader->Peek(info);
    const std::string body = std::string("{\"stream_enabled\":") + (driver_.StreamEnabled() ? "true" : "false") +
                             ",\"shm_path\":\"" + config.shm_path + "\"" + ",\"shm_available\":" +
                             (reader != nullptr ? "true" : "false") + ",\"width\":" + std::to_string(info.width) +
                             ",\"height\":" + std::to_string(info.height) +
                             ",\"preview_step\":" + std::to_string(preview_step_) +
                             ",\"frames_published\":" + std::to_string(have_frame ? info.sequence : 0) + "}\n";
    SendSimpleResponse(client_fd, "200 OK", "application/json", body);
    return;
  }
  if (path == "/") {
    SendSimpleResponse(client_fd, "200 OK", "text/html; charset=utf-8", std::string(kPage, sizeof(kPage) - 1));
    return;
  }
  SendSimpleResponse(client_fd, "404 Not Found", "text/plain", "no such thing\n");
}

void CameraWebServer::ServeStream(int client_fd) {
  // Every connection gets its own reader, so one slow viewer cannot make
  // another skip frames.
  const auto reader = driver_.OpenReader();
  if (reader == nullptr) {
    SendSimpleResponse(client_fd, "503 Service Unavailable", "text/plain",
                       "no camera frames; is cameraProvider.service running?\n");
    return;
  }

  const int nodelay = 1;
  setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
  // No Content-Length: the response is the stream, and it ends when the viewer
  // goes away or the service stops.
  if (!SendAll(client_fd,
               "HTTP/1.1 200 OK\r\n"
               "Content-Type: application/octet-stream\r\n"
               "Cache-Control: no-store\r\n"
               "Connection: close\r\n\r\n")) {
    return;
  }

  std::vector<uint8_t> frame;
  uint32_t width = 0;
  uint32_t height = 0;
  xbot::driver::camera::FrameInfo info{};
  while (running_) {
    if (!reader->GrabI420(frame, preview_step_, width, height, info)) {
      std::this_thread::sleep_for(kPollInterval);
      continue;
    }

    uint8_t header[12];
    const uint32_t fields[3] = {width, height, static_cast<uint32_t>(frame.size())};
    for (size_t field = 0; field < 3; ++field) {
      header[field * 4 + 0] = static_cast<uint8_t>(fields[field] >> 24);
      header[field * 4 + 1] = static_cast<uint8_t>(fields[field] >> 16);
      header[field * 4 + 2] = static_cast<uint8_t>(fields[field] >> 8);
      header[field * 4 + 3] = static_cast<uint8_t>(fields[field]);
    }
    if (!SendAll(client_fd, header, sizeof(header)) || !SendAll(client_fd, frame.data(), frame.size())) {
      return;  // viewer went away
    }
  }
}

}  // namespace goat::camera
