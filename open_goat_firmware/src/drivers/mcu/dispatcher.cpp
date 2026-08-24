// MCU dispatcher - read framed messages from the MCU UART and dispatch
// them to registered handlers. Frame format follows the ECOVACS reverse
// engineering notes: start 0x60, ack, len=data_len+2, cmd0, cmd1, data..., crc8, 0x0A

#include "dispatcher.hpp"

#include <iostream>
#include <cstring>
#include <vector>
#include <utility>
#include <etl/algorithm.h>
#include "misc_utils.h"

namespace xbot::driver::mcu {


bool Dispatcher::StartDriver(std::string path, int baud) {
	DbgAssert(stopped_, "don't start the dispatcher twice");
	if (!stopped_) return false;
	this->mlink = new MCULink(path,baud);
	processing_thread_ = createThread(ThreadEntry, this);
	return true;
}

void Dispatcher::RegisterHandler(uint8_t cmd0, uint8_t cmd1, const MessageHandler &handler) {
	uint16_t key = static_cast<uint16_t>((cmd0 << 8) | cmd1);
	handlers_.emplace_back(key, handler);
}

void Dispatcher::FrameReaderLoop(Dispatcher* instance) {
	// Scan for frames. We return the number of bytes consumed from the front
	while(true){
		if (auto buffer = instance->mlink->read_frame()) {

			uint8_t ack = buffer.value()[1];
			uint8_t cmd0 = buffer.value()[3];
			uint8_t cmd1 = buffer.value()[4];
			size_t data_len = buffer.value().size() - 2;
			const uint8_t *data_ptr = &buffer.value()[5];

			// dispatch
			uint16_t key = static_cast<uint16_t>((cmd0 << 8) | cmd1);
			for (auto &p : instance->handlers_) {
				if (p.first == key) {
					// last two bytes are crc, terminator (0x0A)
					if (p.second) p.second(data_ptr, data_len-2, ack);
				}
			}
		}
	}
}

uint8_t Dispatcher::SendMessage(uint8_t cmd0, uint8_t cmd1, const uint8_t *payload, size_t length) {
	if (!mlink) {
		return 0;
	}

	const uint8_t ack = 0x00;
	mlink->write_frame(cmd0, cmd1, payload, length, ack);

	// The MCU protocol encodes the ACK byte in the frame itself. For now we
	// return a neutral ACK value, since the packet reader handles actual response
	// frames asynchronously via the dispatcher thread.
	return ack;
}

void Dispatcher::ThreadEntry(void* arg) {
	auto inst = static_cast<Dispatcher*>(arg);
	if (inst) FrameReaderLoop(inst);
}

}  // namespace xbot::driver::mcu
