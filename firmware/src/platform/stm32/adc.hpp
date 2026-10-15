// Eight-channel sEMG acquisition: ADC1 in scan mode, triggered by a timer,
// delivered by DMA.
//
// Timer-triggered rather than software-started, because the whole point of the
// acquisition chain is that the interval between samples is uniform. A
// software start puts the sample instant at the mercy of whatever the CPU was
// doing and the jitter that produces looks like signal.
//
// Double-buffered: DMA fills one half while the application reads the other,
// so a frame is never being written and read at the same time. On this part
// that means the buffer has to live somewhere the DMA controller can reach,
// which is not the tightly coupled RAM the stack is in; see the linker script.
#pragma once

#include <array>
#include <cstdint>

#include "pa/config/board.hpp"

namespace pa::platform::adc {

using SampleSet = std::array<int16_t, config::kSemgChannels>;

// Configure the pins, the ADC, the trigger timer and the DMA stream. Returns
// false if calibration or the ADC ready flag does not come back, which is what
// an unpowered analogue supply looks like from here.
bool begin();

// Take the most recent complete set of samples, if there is one that has not
// been read yet. Returns false when the DMA has not finished a set since the
// last call, which at 2 kHz means the caller is early rather than that
// anything is wrong.
bool read(SampleSet& out, uint32_t& timestamp_us);

// Sets that were overwritten before anyone read them. Non-zero means the loop
// is not keeping up with the converter, which is a real fault and is reported
// as one rather than hidden.
uint32_t dropped();

}  // namespace pa::platform::adc
