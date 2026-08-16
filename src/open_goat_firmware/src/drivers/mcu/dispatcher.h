// MCU dispatcher driver - parses framed messages from the MCU UART and
// dispatches them to registered handlers.

#ifndef XBOT_DRIVER_MCU_DISPATCHER_H
#define XBOT_DRIVER_MCU_DISPATCHER_H

#include <etl/delegate.h>
#include "ch.h"
#include "hal.h"

namespace xbot::driver::mcu {

class Dispatcher {
 public:
	using MessageHandler = etl::delegate<void(const uint8_t *payload, size_t length)>;

	Dispatcher() = default;
	~Dispatcher() = default;

	// Start the dispatcher reading from the provided UART at the given baudrate.
	bool StartDriver(UARTDriver *uart, uint32_t baudrate);

	// Register a handler for a two-character command id (cmd0, cmd1).
	void RegisterHandler(uint8_t cmd0, uint8_t cmd1, const MessageHandler &handler);

 protected:
	// Called by the processing thread to handle received bytes
	size_t ProcessBytes(const uint8_t *buffer, size_t len);

 private:
	struct UARTConfigEx : UARTConfig {
		Dispatcher *context;
	};

	static constexpr size_t RECV_BUFFER_SIZE = 512;
	uint8_t recv_buffer1_[RECV_BUFFER_SIZE]{};
	uint8_t recv_buffer2_[RECV_BUFFER_SIZE]{};
	uint8_t *volatile processing_buffer_ = recv_buffer2_;
	volatile size_t processing_buffer_len_ = 0;

	UARTDriver *uart_{};
	UARTConfigEx uart_config_{};

	THD_WORKING_AREA(thd_wa_, 1536){};
	thread_t *processing_thread_ = nullptr;
	volatile bool processing_done_ = true;
	bool stopped_ = true;

	void threadFunc();
	static void threadHelper(void *instance);

	// Handler registry keyed by combined cmd0/cmd1
	etl::delegate<void(const uint8_t *, size_t)> default_handler_{};
	// Use a simple fixed map keyed by uint16_t
	std::vector<std::pair<uint16_t, MessageHandler>> handlers_{};
};

}  // namespace xbot::driver::mcu

#endif  // XBOT_DRIVER_MCU_DISPATCHER_H

