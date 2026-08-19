#include <iostream>
#include <vector>
#include <string>
#include <optional>
#include <cstdint>
#include <chrono>
#include <thread>
#include <algorithm>
#include <ulog.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>


class SerialDriver {
private:
    std::string path;
    int baud;
    int fd = -1;

    speed_t get_baud_rate_constant(int baud_rate) const {
        switch (baud_rate) {
            case 9600:   return B9600;
            case 57600:  return B57600;
            case 115200: return B115200;
            default:     return B115200;
        }
    }

public:
    SerialDriver(std::string path, int baud) 
        : path(std::move(path)), baud(baud) {
            open();
        }

    ~SerialDriver() {
        close();
    }

    // Prevent copying to avoid duplicate file descriptor management
    SerialDriver(const SerialDriver&) = delete;
    SerialDriver& operator=(const SerialDriver&) = delete;

    // Enable move semantics
    SerialDriver(SerialDriver&& other) noexcept 
        : path(std::move(other.path)), baud(other.baud), fd(other.fd) {
        other.fd = -1;
    }

    SerialDriver& operator=(SerialDriver&& other) noexcept {
        if (this != &other) {
            close();
            path = std::move(other.path);
            baud = other.baud;
            fd = other.fd;
            other.fd = -1;
        }
        return *this;
    }

    bool open() {
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            ULOG_ERROR("failed to open TTY for SerialDriver!");
            return false;
        }

        struct termios attr{};
        if (tcgetattr(fd, &attr) != 0) {
            close();
            ULOG_ERROR("failed to get TTY attributes for SerialDriver!");
            return false;
        }

        speed_t speed = get_baud_rate_constant(baud);

        attr.c_iflag = 0;                                 // raw input
        attr.c_oflag = 0;                                 // raw output
        attr.c_cflag = CS8 | CREAD | CLOCAL;              // 8N1, enable receiver, ignore modem control
        attr.c_lflag = 0;                                 // raw mode (non-canonical)
        
        cfsetispeed(&attr, speed);
        cfsetospeed(&attr, speed);

        attr.c_cc[VMIN] = 0;                              // non-blocking read
        attr.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &attr) != 0) {
            ULOG_ERROR("failed to set TTY attributes for SerialDriver!");
            close();
            return false;
        }

        return true;
    }

    void close() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    size_t write(const std::vector<uint8_t>& data) {
        if (fd < 0) return -1;
        return ::write(fd, data.data(), data.size());
    }

    size_t write(const uint8_t* data, size_t size) {
        if (fd < 0) return -1;
        return ::write(fd, data, size);
    }

    size_t read(uint8_t* buffer, size_t size) {
        if (fd < 0) return -1;
        return ::read(fd, buffer, size);
    }
};
