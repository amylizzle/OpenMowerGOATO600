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
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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

        attr.c_cc[VMIN] = 0;                              // non-blocking read
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


    void print_hex(const std::vector<uint8_t>& data) {
        // Save original formatting flags so we don't pollute std::cout later
        std::ios state(nullptr);
        state.copyfmt(std::cout);

        std::cout << std::hex << std::setfill('0');
        
        for (uint8_t byte : data) {
            // Cast to unsigned to prevent char printing and avoid sign extension
            std::cout << std::setw(2) << static_cast<unsigned>(byte) << ' ';
        }
        std::cout << '\n';

        // Restore original formatting
        std::cout.copyfmt(state);
    }

    ssize_t write(const std::vector<uint8_t>& data) {
        if (fd < 0) return -1;
        print_hex(data);
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
        if (fd < 0) {
            ULOG_ERROR("MCU LOST TTY FILE, REOPENING");
            open();
            return std::nullopt;
        }

        std::vector<uint8_t> buf;
        using clock = std::chrono::steady_clock;
        
        auto end_time = clock::now() + std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(timeout)
        );

        while (clock::now() < end_time) {
            uint8_t chunk[256];
            ssize_t bytes_read = ::read(fd, chunk, sizeof(chunk));
            
            if (bytes_read > 0) {
                buf.insert(buf.end(), chunk, chunk + bytes_read);
            }

            while (true) {
                auto it = std::find(buf.begin(), buf.end(), static_cast<uint8_t>(0x60));
                if (it == buf.end()) {
                    buf.clear(); // No delimiter left in buffer
                    break;
                }

                size_t i = std::distance(buf.begin(), it);
                if (buf.size() - i < 3) {
                    break;
                }

                uint8_t ln = buf[i + 2];
                size_t total = static_cast<size_t>(ln) + 5;
                if (buf.size() - i < total) {
                    break;
                }

                std::vector<uint8_t> frame(buf.begin() + i, buf.begin() + i + total);

                bool terminator_ok = (frame.back() == 0x0A);
                uint8_t expected_crc = crc8(frame.data(), frame.size() - 2);
                uint8_t got_crc = frame[frame.size() - 2];
                bool crc_ok = (expected_crc == got_crc);

                if (terminator_ok && crc_ok) {
                    buf.erase(buf.begin(), buf.begin() + i + total);
                    return frame;
                }

                // False positive resync logic
                // std::cout << "desync/bad frame at offset " << i
                //           << " (terminator_ok=" << (terminator_ok ? "True" : "False")
                //           << ", crc_ok=" << (crc_ok ? "True" : "False")
                //           << ", expected_crc=" << static_cast<int>(expected_crc)
                //           << ", got=" << static_cast<int>(got_crc) 
                //           << ") - resyncing\n";

                buf.erase(buf.begin() + i, buf.begin() + i + 1);
            }

            std::this_thread::sleep_for(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(poll))
            );
        }

        return std::nullopt;
    }
};
