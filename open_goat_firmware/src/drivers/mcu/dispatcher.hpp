// MCU dispatcher driver - parses framed messages from the MCU UART and
// dispatches them to registered handlers.

#ifndef XBOT_DRIVER_MCU_DISPATCHER_H
#define XBOT_DRIVER_MCU_DISPATCHER_H

#include <etl/delegate.h>
#include "misc_utils.h"
#include "MCU_parser.cpp"

namespace xbot::driver::mcu {

class Dispatcher {
 public:
	using MessageHandler = etl::delegate<void(const uint8_t *payload, size_t length, uint8_t ack)>;

	Dispatcher() = default;
	~Dispatcher() = default;

	// Start the dispatcher reading from the provided UART at the given baudrate.
	bool StartDriver(std::string path, int baud);

	// Register a handler for a two-character command id (cmd0, cmd1).
	void RegisterHandler(uint8_t cmd0, uint8_t cmd1, const MessageHandler &handler);

	// Encode and send a frame, return ack byte
	uint8_t SendMessage(uint8_t cmd0, uint8_t cmd1, const uint8_t *payload, size_t length);

 protected:
	MCULink* mlink{};
	bool stopped_ = true;
	thread_t *processing_thread_ = nullptr;
	// Handler registry keyed by combined cmd0/cmd1
	MessageHandler default_handler_{};
	// Use a simple fixed map keyed by uint16_t
	std::vector<std::pair<uint16_t, MessageHandler>> handlers_{};

	// Thread entry for reading frames from the MCU link
	static void FrameReaderLoop(Dispatcher* instance);

	// Adapter matching createThread(void(*)(void*), void*)
	static void ThreadEntry(void* arg);
};

}  // namespace xbot::driver::mcu

#endif  // XBOT_DRIVER_MCU_DISPATCHER_H

