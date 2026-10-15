// Pin configuration, as a few inline helpers rather than a driver.
//
// firmware/docs/PINOUT.md is the authority on which pin does what; this file
// is only the mechanism for saying it.
#pragma once

#include <cstdint>

#include "registers.hpp"

namespace pa::platform::gpio {

enum class Mode : uint32_t { kInput = 0, kOutput = 1, kAlternate = 2, kAnalogue = 3 };
enum class Pull : uint32_t { kNone = 0, kUp = 1, kDown = 2 };
enum class Speed : uint32_t { kLow = 0, kMedium = 1, kHigh = 2, kVeryHigh = 3 };
enum class Type : uint32_t { kPushPull = 0, kOpenDrain = 1 };

inline void configure(GpioRegisters* port, uint32_t pin, Mode mode, Pull pull = Pull::kNone,
                      Speed speed = Speed::kHigh, Type type = Type::kPushPull,
                      uint32_t alternate = 0) {
    const uint32_t two_bit = pin * 2;
    port->MODER = (port->MODER & ~(3u << two_bit)) | (static_cast<uint32_t>(mode) << two_bit);
    port->PUPDR = (port->PUPDR & ~(3u << two_bit)) | (static_cast<uint32_t>(pull) << two_bit);
    port->OSPEEDR = (port->OSPEEDR & ~(3u << two_bit)) | (static_cast<uint32_t>(speed) << two_bit);
    port->OTYPER = (port->OTYPER & ~(1u << pin)) | (static_cast<uint32_t>(type) << pin);
    if (mode == Mode::kAlternate) {
        const uint32_t index = pin >> 3;
        const uint32_t shift = (pin & 7u) * 4;
        port->AFR[index] = (port->AFR[index] & ~(0xFu << shift)) | (alternate << shift);
    }
}

inline void set(GpioRegisters* port, uint32_t pin) { port->BSRR = 1u << pin; }
inline void clear(GpioRegisters* port, uint32_t pin) { port->BSRR = 1u << (pin + 16); }
inline bool read(GpioRegisters* port, uint32_t pin) { return (port->IDR & (1u << pin)) != 0; }

}  // namespace pa::platform::gpio
