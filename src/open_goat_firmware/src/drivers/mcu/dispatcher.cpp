// MCU dispatcher - read framed messages from the MCU UART and dispatch
// them to registered handlers. Frame format follows the ECOVACS reverse
// engineering notes: start 0x60, ack, len=data_len+2, cmd0, cmd1, data..., crc8, 0x0A

#include "dispatcher.h"

#include <etl/algorithm.h>
#include "posix_ch.h"
#include <cstring>
#include <vector>
#include <utility>

namespace xbot::driver::mcu {

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

bool Dispatcher::StartDriver(UARTDriver *uart, uint32_t baudrate) {
	chDbgAssert(stopped_, "don't start the dispatcher twice");
	chDbgAssert(uart != nullptr, "need to provide a driver");
	if (!stopped_) return false;

	// ensure CRC table is built once
	static bool crc_ready = false;
	if (!crc_ready) {
		build_crc_table();
		crc_ready = true;
	}

	uart_ = uart;
	uart_config_.speed = baudrate;
	uart_config_.context = this;

	uart_config_.rxend_cb = [](UARTDriver *uartp) {
		chSysLockFromISR();
		Dispatcher *instance = reinterpret_cast<const UARTConfigEx *>(uartp->config)->context;
		chDbgAssert(instance != nullptr, "instance cannot be null!");
		if (!instance->processing_done_) {
			uint8_t *next_recv_buffer = (instance->processing_buffer_ == instance->recv_buffer1_) ? instance->recv_buffer2_ : instance->recv_buffer1_;
			uartStartReceiveI(uartp, RECV_BUFFER_SIZE, next_recv_buffer);
		} else {
			uint8_t *next_recv_buffer = instance->processing_buffer_;
			uartStartReceiveI(uartp, RECV_BUFFER_SIZE, next_recv_buffer);
			instance->processing_buffer_ = (instance->processing_buffer_ == instance->recv_buffer1_) ? instance->recv_buffer2_ : instance->recv_buffer1_;
			instance->processing_buffer_len_ = RECV_BUFFER_SIZE;
			instance->processing_done_ = false;
			if (instance->processing_thread_) {
				chEvtSignalI(instance->processing_thread_, 1);
			}
		}
		chSysUnlockFromISR();
	};

	bool started = uartStart(uart, &uart_config_) == MSG_OK;
	if (!started) return false;

	stopped_ = false;
	processing_thread_ = chThdCreateStatic(&thd_wa_, sizeof(thd_wa_), NORMALPRIO, threadHelper, this);
	uartStartReceive(uart, RECV_BUFFER_SIZE, recv_buffer1_);
	return true;
}

void Dispatcher::RegisterHandler(uint8_t cmd0, uint8_t cmd1, const MessageHandler &handler) {
	uint16_t key = static_cast<uint16_t>((cmd0 << 8) | cmd1);
	handlers_.emplace_back(key, handler);
}

size_t Dispatcher::ProcessBytes(const uint8_t *buffer, size_t len) {
	// Scan for frames. We return the number of bytes consumed from the front
	size_t i = 0;
	while (i + 7 <= len) {
		// look for start delimiter
		if (buffer[i] != 0x60) {
			++i;
			continue;
		}
		if (i + 3 >= len) break; // need ack + len
		uint8_t ack = buffer[i + 1];
		uint8_t length = buffer[i + 2]; // length = data_len + 2
		size_t frame_len = static_cast<size_t>(length) + 7; // total
		if (i + frame_len > len) break; // incomplete

		// verify trailer
		if (buffer[i + frame_len - 1] != 0x0A) {
			// malformed, skip start byte
			++i;
			continue;
		}

		uint8_t cmd0 = buffer[i + 3];
		uint8_t cmd1 = buffer[i + 4];
		size_t data_len = (size_t)length - 2;
		const uint8_t *data_ptr = &buffer[i + 5];
		uint8_t frame_crc = buffer[i + 5 + data_len];

		// compute crc over start..(cmd1+data)
		uint8_t calc = crc8(&buffer[i], 5 + data_len);
		if (calc != frame_crc) {
			// CRC mismatch, skip this start and continue
			++i;
			continue;
		}

		// dispatch
		uint16_t key = static_cast<uint16_t>((cmd0 << 8) | cmd1);
		bool dispatched = false;
		for (auto &p : handlers_) {
			if (p.first == key) {
				if (p.second) p.second(data_ptr, data_len);
				dispatched = true;
			}
		}
		if (!dispatched && default_handler_) {
			default_handler_(data_ptr, data_len);
		}

		// advance past the frame
		i += frame_len;
	}
	return i;
}

void Dispatcher::threadFunc() {
	uint32_t last_ndtr = 0;
	while (!stopped_) {
		bool timeout = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(25)) == 0;
		if (timeout) {
			if (last_ndtr != uart_->dmarx->stream->NDTR) {
				last_ndtr = uart_->dmarx->stream->NDTR;
				continue;
			}
			chSysLock();
			if (processing_done_) {
				size_t not_received_len = uartStopReceiveI(uart_);
				if (not_received_len != UART_ERR_NOT_ACTIVE) {
					processing_buffer_len_ = RECV_BUFFER_SIZE - not_received_len;
				} else {
					processing_buffer_len_ = 0;
				}
				uint8_t *next_recv_buffer = processing_buffer_;
				uartStartReceiveI(uart_, RECV_BUFFER_SIZE, next_recv_buffer);
				processing_buffer_ = (processing_buffer_ == recv_buffer1_) ? recv_buffer2_ : recv_buffer1_;
				processing_done_ = false;
			}
			chSysUnlock();
		}
		if (processing_buffer_len_ > 0) {
			// Process the bytes in place, discard consumed prefix by memmove
			size_t consumed = ProcessBytes(processing_buffer_, processing_buffer_len_);
			if (consumed > 0 && consumed < processing_buffer_len_) {
				size_t remain = processing_buffer_len_ - consumed;
				memmove(processing_buffer_, processing_buffer_ + consumed, remain);
				processing_buffer_len_ = remain;
			} else {
				processing_buffer_len_ = 0;
			}
		}
		last_ndtr = 0;
		processing_buffer_len_ = 0;
		processing_done_ = true;
	}
}

void Dispatcher::threadHelper(void *instance) {
	chRegSetThreadName("McuDispatcher");
	auto *drv = static_cast<Dispatcher *>(instance);
	drv->threadFunc();
}

}  // namespace xbot::driver::mcu

