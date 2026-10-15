// Six servo pulse outputs, from two timers.
//
// TIM1 carries four channels and TIM3 the other two, because no timer on this
// part has six compare channels. Both are configured identically and started
// from the same instruction, so the six pulses stay in phase with each other:
// six servos all starting their pulse at the same instant is what makes the
// current draw predictable, which is what the thermal budget assumes.
//
// Disabling the outputs stops the pulse train rather than sending a centre
// pulse. A servo with no pulses relaxes and that is the behaviour a lost host
// should produce.
#pragma once

#include <array>
#include <cstdint>

#include "pa/config/board.hpp"

namespace pa::platform::pwm {

bool begin();

// Set all six pulse widths, in microseconds. Values outside the electrical
// range are clamped here as well as in the servo driver: this is the last
// place before a timer register and a pulse outside the range is the one
// mistake that can stall a motor.
void set_pulses(const std::array<float, config::kServoChannels>& microseconds);

// Stop the pulse train. The servos go limp.
void disable();
void enable();
bool enabled();

}  // namespace pa::platform::pwm
