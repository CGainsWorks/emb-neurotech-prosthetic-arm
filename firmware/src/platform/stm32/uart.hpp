// USART1 to the host, DMA out and interrupt in.
//
// The board sends about 12 kB a second of sEMG frames and telemetry. Sending
// that a byte at a time from the main loop would put a UART wait inside the
// control period; DMA out of a ring buffer keeps the loop free and makes the
// transmit cost independent of how much is being sent.
//
// Receive is interrupt driven and byte at a time, straight into the frame
// decoder, because commands are short and infrequent and a DMA on the receive
// side would need an idle-line timeout to know when a frame ended, which the
// framing already tells it.
#pragma once

#include <cstddef>
#include <cstdint>

#include "pa/protocol/frame.hpp"

namespace pa::platform::uart {

bool begin();

// Queue a frame. Returns false when the ring is full, which is the honest
// answer: the alternative is blocking inside the control loop and a dropped
// telemetry frame costs nothing while a late servo update costs a movement.
bool send(const uint8_t* data, std::size_t length);

// Frames dropped because the ring was full. Reported in the status message.
uint32_t dropped();

// Pull the next decoded command, if one has arrived. The decoder runs in the
// receive interrupt so a frame is complete by the time the loop asks.
bool poll(protocol::DecodedFrame& frame);

// Counters from the decoder, for the fault layer.
uint32_t link_errors();

}  // namespace pa::platform::uart
