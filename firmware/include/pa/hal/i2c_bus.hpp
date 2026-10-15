// The only thing the IMU driver knows about the hardware underneath it.
//
// Keeping the bus behind an interface is what lets the whole driver, register
// map and all, run in the host test suite against a simulated device. Without
// it none of the IMU code could be tested at all until a board existed, which
// on this project would mean never.
#pragma once

#include <cstddef>
#include <cstdint>

namespace pa::hal {

enum class I2cStatus : uint8_t {
    kOk = 0,
    kNack,        // no device acknowledged the address
    kBusError,    // arbitration lost, or the bus is stuck low
    kTimeout,
};

class I2cBus {
  public:
    virtual ~I2cBus() = default;

    // Write `length` bytes to `address`, starting at register `reg`.
    virtual I2cStatus write(uint8_t address, uint8_t reg, const uint8_t* data,
                            std::size_t length) = 0;

    // Read `length` bytes from `address`, starting at register `reg`.
    virtual I2cStatus read(uint8_t address, uint8_t reg, uint8_t* data, std::size_t length) = 0;

    I2cStatus write_byte(uint8_t address, uint8_t reg, uint8_t value) {
        return write(address, reg, &value, 1);
    }

    I2cStatus read_byte(uint8_t address, uint8_t reg, uint8_t& value) {
        return read(address, reg, &value, 1);
    }
};

}  // namespace pa::hal
