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
	this->RegisterHandler(
          static_cast<uint8_t>('T'), static_cast<uint8_t>('B'), // Log text message
          etl::delegate<void(const uint8_t *, size_t, uint8_t)>::create<Dispatcher, &Dispatcher::OnTBMessage>(*this));
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
			auto val = buffer.value();
			uint8_t ack = val[1];
			uint8_t cmd0 = val[3];
			uint8_t cmd1 = val[4];
			// first 5 bytes are [0x60][ack][len][cmd0][cmd1], last two bytes are [crc],[0x0A]
			size_t data_len = (val.size() - 5) - 2;
			const uint8_t *data_ptr = &val[5];

			// dispatch
			bool dispatched = false;
			uint16_t key = static_cast<uint16_t>((cmd0 << 8) | cmd1);
			for (auto &p : instance->handlers_) {
				if (p.first == key) {
					if (p.second) {
						p.second(data_ptr, data_len, ack);
						dispatched = true;
					}
						
				}
			}
			if(!dispatched){
				ULOG_WARNING("UNHANDLED MCU MESSAGE %c%c",cmd0,cmd1);
			}
		} else {
			std::this_thread::sleep_for(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(0.05))
            );
		}
	}
}

uint8_t Dispatcher::SendMessage(uint8_t cmd0, uint8_t cmd1, const uint8_t *payload, size_t length) {
	if (!mlink) {
		return 0;
	}
	mlink->write_frame(cmd0, cmd1, payload, length, ack);
	return ack;
}

void Dispatcher::ThreadEntry(void* arg) {
	auto inst = static_cast<Dispatcher*>(arg);
	if (inst) FrameReaderLoop(inst);
}

void Dispatcher::OnTBMessage(const uint8_t *payload, size_t length, uint8_t ack) {
    (void) ack;
	//-2 for the \r\n which ULOG already provides
	ULOG_INFO("MCU Message: %s", std::string(reinterpret_cast<const char*>(payload), length-2).c_str());
}

}  // namespace xbot::driver::mcu
