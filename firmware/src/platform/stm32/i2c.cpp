#include "i2c.hpp"

#include "gpio.hpp"
#include "registers.hpp"
#include "system.hpp"

namespace pa::platform {
namespace {

// PB6 and PB7, alternate function 4.
constexpr uint32_t kSclPin = 6;
constexpr uint32_t kSdaPin = 7;

// Long enough to cover a stretched clock on a healthy slave, short enough that
// a stuck bus is reported inside one control period.
constexpr uint32_t kTransferTimeoutUs = 2000;

// TIMINGR for 400 kHz from a 100 MHz peripheral clock, from the values the
// reference manual's own table gives for this combination: prescaler 3,
// SCLDEL 3, SDADEL 0, SCLH 3, SCLL 9.
constexpr uint32_t kTiming400k = (3u << 28) | (3u << 20) | (0u << 16) | (3u << 8) | 9u;

bool spin_until(volatile uint32_t& reg, uint32_t mask, bool set, uint32_t timeout_us) {
    const uint32_t start = micros();
    while (((reg & mask) != 0) != set) {
        if (micros() - start > timeout_us) return false;
    }
    return true;
}

}  // namespace

bool I2cBus1::begin() {
    RCC->AHB4ENR |= (1u << 1);   // GPIOB
    RCC->APB1LENR |= (1u << 21); // I2C1
    dsb();

    gpio::configure(GPIOB, kSclPin, gpio::Mode::kAlternate, gpio::Pull::kUp,
                    gpio::Speed::kHigh, gpio::Type::kOpenDrain, 4);
    gpio::configure(GPIOB, kSdaPin, gpio::Mode::kAlternate, gpio::Pull::kUp,
                    gpio::Speed::kHigh, gpio::Type::kOpenDrain, 4);

    I2C1->CR1 = 0;  // PE off while TIMINGR is written
    I2C1->TIMINGR = kTiming400k;
    I2C1->CR1 = 1u;  // PE

    // BUSY at this point means a slave is holding the bus from before the
    // reset, which recover() is for.
    if (I2C1->ISR & (1u << 15)) {
        recover();
        return (I2C1->ISR & (1u << 15)) == 0;
    }
    return true;
}

void I2cBus1::recover() {
    I2C1->CR1 = 0;
    gpio::configure(GPIOB, kSclPin, gpio::Mode::kOutput, gpio::Pull::kUp, gpio::Speed::kHigh,
                    gpio::Type::kOpenDrain);
    gpio::configure(GPIOB, kSdaPin, gpio::Mode::kInput, gpio::Pull::kUp);

    // Nine pulses is one byte plus the acknowledge, which is the most a slave
    // can be waiting on.
    for (int pulse = 0; pulse < 9; ++pulse) {
        gpio::clear(GPIOB, kSclPin);
        delay_us(5);
        gpio::set(GPIOB, kSclPin);
        delay_us(5);
        if (gpio::read(GPIOB, kSdaPin)) break;
    }

    // A stop condition, so whatever was listening knows the transfer is over.
    gpio::configure(GPIOB, kSdaPin, gpio::Mode::kOutput, gpio::Pull::kUp, gpio::Speed::kHigh,
                    gpio::Type::kOpenDrain);
    gpio::clear(GPIOB, kSdaPin);
    delay_us(5);
    gpio::set(GPIOB, kSclPin);
    delay_us(5);
    gpio::set(GPIOB, kSdaPin);
    delay_us(5);

    gpio::configure(GPIOB, kSclPin, gpio::Mode::kAlternate, gpio::Pull::kUp, gpio::Speed::kHigh,
                    gpio::Type::kOpenDrain, 4);
    gpio::configure(GPIOB, kSdaPin, gpio::Mode::kAlternate, gpio::Pull::kUp, gpio::Speed::kHigh,
                    gpio::Type::kOpenDrain, 4);

    I2C1->TIMINGR = kTiming400k;
    I2C1->CR1 = 1u;
}

hal::I2cStatus I2cBus1::write(uint8_t address, uint8_t reg, const uint8_t* data,
                              std::size_t length) {
    if (length > 255) return hal::I2cStatus::kBusError;

    I2C1->ICR = 0x3F38;  // clear the error and stop flags
    I2C1->CR2 = (static_cast<uint32_t>(address) << 1)
              | ((length + 1) << 16)
              | (1u << 25)   // AUTOEND
              | (1u << 13);  // START

    if (!spin_until(I2C1->ISR, 1u << 1, true, kTransferTimeoutUs)) {  // TXIS
        ++timeouts_;
        return (I2C1->ISR & (1u << 4)) ? hal::I2cStatus::kNack : hal::I2cStatus::kTimeout;
    }
    I2C1->TXDR = reg;

    for (std::size_t i = 0; i < length; ++i) {
        if (!spin_until(I2C1->ISR, 1u << 1, true, kTransferTimeoutUs)) {
            ++timeouts_;
            return (I2C1->ISR & (1u << 4)) ? hal::I2cStatus::kNack : hal::I2cStatus::kTimeout;
        }
        I2C1->TXDR = data[i];
    }

    if (!spin_until(I2C1->ISR, 1u << 5, true, kTransferTimeoutUs)) {  // STOPF
        ++timeouts_;
        return hal::I2cStatus::kTimeout;
    }
    I2C1->ICR = (1u << 5);
    return hal::I2cStatus::kOk;
}

hal::I2cStatus I2cBus1::read(uint8_t address, uint8_t reg, uint8_t* data, std::size_t length) {
    if (length == 0 || length > 255) return hal::I2cStatus::kBusError;

    // Register address first, without a stop, so the repeated start that
    // follows leaves the device's address pointer where it was put.
    I2C1->ICR = 0x3F38;
    I2C1->CR2 = (static_cast<uint32_t>(address) << 1) | (1u << 16) | (1u << 13);
    if (!spin_until(I2C1->ISR, 1u << 1, true, kTransferTimeoutUs)) {
        ++timeouts_;
        return (I2C1->ISR & (1u << 4)) ? hal::I2cStatus::kNack : hal::I2cStatus::kTimeout;
    }
    I2C1->TXDR = reg;
    if (!spin_until(I2C1->ISR, 1u << 6, true, kTransferTimeoutUs)) {  // TC
        ++timeouts_;
        return hal::I2cStatus::kTimeout;
    }

    I2C1->CR2 = (static_cast<uint32_t>(address) << 1)
              | (static_cast<uint32_t>(length) << 16)
              | (1u << 10)   // RD_WRN
              | (1u << 25)   // AUTOEND
              | (1u << 13);  // repeated START

    for (std::size_t i = 0; i < length; ++i) {
        if (!spin_until(I2C1->ISR, 1u << 2, true, kTransferTimeoutUs)) {  // RXNE
            ++timeouts_;
            return (I2C1->ISR & (1u << 4)) ? hal::I2cStatus::kNack : hal::I2cStatus::kTimeout;
        }
        data[i] = static_cast<uint8_t>(I2C1->RXDR);
    }

    if (!spin_until(I2C1->ISR, 1u << 5, true, kTransferTimeoutUs)) {
        ++timeouts_;
        return hal::I2cStatus::kTimeout;
    }
    I2C1->ICR = (1u << 5);
    return hal::I2cStatus::kOk;
}

}  // namespace pa::platform
