// MCU dispatcher - read framed messages from the MCU UART and dispatch
// them to registered handlers. Frame format follows the ECOVACS reverse
// engineering notes: start 0x60, ack, len=data_len+2, cmd0, cmd1, data..., crc8, 0x0A

#include "dispatcher.hpp"

#include <iostream>
#include <cstring>
#include <vector>
#include <utility>
#include <etl/algorithm.h>
#include "posix_ch.h"

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
					if (p.second) p.second(data_ptr, data_len, ack);
				}
			}
		}
	}
}

void Dispatcher::ThreadEntry(void* arg) {
	auto inst = static_cast<Dispatcher*>(arg);
	if (inst) FrameReaderLoop(inst);
}

}  // namespace xbot::driver::mcu
