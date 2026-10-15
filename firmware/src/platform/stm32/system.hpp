// Clock tree, the microsecond clock and the busy-wait delay.
#pragma once

#include <cstdint>

namespace pa::platform {

// System clock the board is configured for, in hertz.
//
// 400 MHz, not the 480 MHz the part is marketed at. 480 needs VOS0 and the
// overdrive sequence and it is only available on revision V silicon; 400 MHz
// at VOS1 works on every revision and costs about a fifth of the classifier's
// inference budget. pa/config/board.hpp records what that does to the
// end-to-end latency estimate, which is nothing that matters.
inline constexpr uint32_t kSysClockHz = 400'000'000u;
inline constexpr uint32_t kAhbClockHz = kSysClockHz / 2;   // 200 MHz
inline constexpr uint32_t kApbClockHz = kAhbClockHz / 2;   // 100 MHz
// Timers on APB1 and APB2 run at twice the bus clock whenever the bus
// prescaler is not one, which it is not here.
inline constexpr uint32_t kTimerClockHz = kApbClockHz * 2;  // 200 MHz

// Configure the flash wait states, the voltage scale, the PLL and the bus
// prescalers, then switch the system clock over. Returns false if the
// oscillator or the PLL fails to lock, in which case the caller is still on
// the 64 MHz internal oscillator and should say so rather than carry on
// pretending the timing is what it planned.
bool configure_clocks();

// Free-running microsecond counter, from the cycle counter in the debug unit.
// One load to read and no timer spent on it.
uint32_t micros();

void delay_us(uint32_t microseconds);
void delay_ms(uint32_t milliseconds);

// Milliseconds since start-up, derived from the same counter. Wraps every
// 49 days; every comparison against it in this firmware is written to survive
// that.
uint32_t millis();

}  // namespace pa::platform
