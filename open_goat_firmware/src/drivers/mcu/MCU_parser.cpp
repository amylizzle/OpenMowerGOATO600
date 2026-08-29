#include <iostream>
#include <vector>
#include <iomanip>
#include <cstdint>
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

// CRC8 config 
static constexpr uint8_t CRC_POLY = 0x07;
static constexpr uint8_t CRC_INIT = 0x00;
static constexpr uint8_t CRC_XOROUT = 0x00;

static uint8_t crc_table[256];

static void build_crc_table() {
	for (int i = 0; i < 256; ++i) {
		uint8_t v = static_cast<uint8_t>(i);
		for (int b = 0; b < 8; ++b) {
			v = (v & 0x80) ? static_cast<uint8_t>((v << 1) ^ CRC_POLY) : static_cast<uint8_t>(v << 1);
		}
		crc_table[i] = v;
	}
}

static uint8_t crc8(const uint8_t *data, size_t len) {
	uint8_t crc = CRC_INIT;
	for (size_t i = 0; i < len; ++i) {
		crc = crc_table[(crc ^ data[i]) & 0xFF];
	}
	return static_cast<uint8_t>(crc ^ CRC_XOROUT);
}

class MCULink {
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
    MCULink(std::string path, int baud) 
        : path(std::move(path)), baud(baud) {
            	// ensure CRC table is built once
            static bool crc_ready = false;
            if (!crc_ready) {
                build_crc_table();
                crc_ready = true;
            }
            open();
        }

    ~MCULink() {
        close();
    }

    // Prevent copying to avoid duplicate file descriptor management
    MCULink(const MCULink&) = delete;
    MCULink& operator=(const MCULink&) = delete;

    // Enable move semantics
    MCULink(MCULink&& other) noexcept 
        : path(std::move(other.path)), baud(other.baud), fd(other.fd) {
        other.fd = -1;
    }

    MCULink& operator=(MCULink&& other) noexcept {
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
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY);
        if (fd < 0) {
            ULOG_ERROR("failed to open TTY for MCULink!");
            return false;
        }

        struct termios attr{};
        if (tcgetattr(fd, &attr) != 0) {
            close();
            ULOG_ERROR("failed to get TTY attributes for MCULink!");
            return false;
        }

        speed_t speed = get_baud_rate_constant(baud);

        attr.c_iflag = 0;                                 // raw input
        attr.c_oflag = 0;                                 // raw output
        attr.c_cflag = CS8 | CREAD | CLOCAL;              // 8N1, enable receiver, ignore modem control
        attr.c_lflag = 0;                                 // raw mode (non-canonical)
        
        cfsetispeed(&attr, speed);
        cfsetospeed(&attr, speed);

        attr.c_cc[VMIN] = 1;                              // blocking read, wait for at least one byte
        attr.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &attr) != 0) {
            ULOG_ERROR("failed to set TTY attributes for MCULink!");
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


    ssize_t write(const std::vector<uint8_t>& data) {
        if (fd < 0) return -1;
        return ::write(fd, data.data(), data.size());
    }

    void write_frame(uint8_t cmd0, uint8_t cmd1, const uint8_t* data, size_t size, uint8_t ack) {
        // write [0x60][ack][len][cmd0][cmd1][data...][crc][0x0A]
        // len is payload bytes + cmd0 + cmd1 (i.e. data size + 2)
        std::vector<uint8_t> frame;
        frame.reserve(1 + 1 + 1 + 1 + 1 + size + 1 + 1);

        frame.push_back(0x60);
        frame.push_back(ack);
        frame.push_back(static_cast<uint8_t>(size + 2u));
        frame.push_back(cmd0);
        frame.push_back(cmd1);

        if (data != nullptr && size > 0) {
            frame.insert(frame.end(), data, data + size);
        }

        uint8_t crc = crc8(frame.data(), frame.size());
        frame.push_back(crc);
        frame.push_back(0x0A);

        this->write(frame);
    }

    std::optional<std::vector<uint8_t>> read_frame(double timeout = 1.0, double poll = 0.05) {
        (void)timeout;
        (void)poll;

        if (fd < 0) {
            ULOG_ERROR("MCU LOST TTY FILE, REOPENING");
            open();
            return std::nullopt;
        }

        std::vector<uint8_t> frame;
        while (true) {
            uint8_t byte = 0;
            const ssize_t n = ::read(fd, &byte, 1);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                ULOG_ERROR("MCU read failed: %s", ::strerror(errno));
                return std::nullopt;
            }
            if (n == 0) {
                continue;
            }

            if (frame.empty()) {
                if (byte != 0x60) {
                    continue;
                }
                frame.push_back(byte);
                continue;
            }

            if (frame.size() == 1) {
                frame.push_back(byte);
                continue;
            }

            if (frame.size() == 2) {
                frame.push_back(byte);
                const uint8_t length = frame[2];
                if (length < 2) {
                    frame.clear();
                    continue;
                }
                continue;
            }

            const size_t expected_total = static_cast<size_t>(frame[2]) + 5u;
            frame.push_back(byte);

            if (frame.size() < expected_total) {
                continue;
            }

            if (frame.size() > expected_total) {
                auto next_start = std::find(frame.begin() + 1, frame.end(), static_cast<uint8_t>(0x60));
                if (next_start != frame.end()) {
                    const auto offset = std::distance(frame.begin(), next_start);
                    frame.erase(frame.begin(), frame.begin() + static_cast<std::ptrdiff_t>(offset));
                    continue;
                }
                frame.clear();
                continue;
            }

            const uint8_t got_crc = frame[frame.size() - 2];
            const uint8_t terminator = frame.back();
            const uint8_t expected_crc = crc8(frame.data(), frame.size() - 2);

            if (terminator == 0x0A && expected_crc == got_crc) {
                return frame;
            }

            auto next_start = std::find(frame.begin() + 1, frame.end(), static_cast<uint8_t>(0x60));
            if (next_start != frame.end()) {
                const auto offset = std::distance(frame.begin(), next_start);
                frame.erase(frame.begin(), frame.begin() + static_cast<std::ptrdiff_t>(offset));
                continue;
            }

            frame.clear();
        }
    }
};
