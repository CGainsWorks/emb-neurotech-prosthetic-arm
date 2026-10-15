// I2C1, implementing the bus interface the IMU driver was written against.
//
// Polled rather than interrupt driven. The IMU is read once per ten
// milliseconds and a twelve-byte burst at 400 kHz takes about 300
// microseconds, which fits inside its slot with room left; an interrupt state
// machine would buy back a third of a percent of the CPU and cost a class of
// bug that only appears when the bus glitches.
//
// Every transfer has a timeout. A held-low SDA is the classic I2C failure and
// a driver that waits for it forever takes the whole arm with it.
#pragma once

#include "pa/hal/i2c_bus.hpp"

namespace pa::platform {

class I2cBus1 : public hal::I2cBus {
  public:
    // Configure the pins and the peripheral for 400 kHz fast mode. Returns
    // false if the bus is not idle at the end, which is what a stuck slave
    // looks like.
    bool begin();

    hal::I2cStatus write(uint8_t address, uint8_t reg, const uint8_t* data,
                         std::size_t length) override;
    hal::I2cStatus read(uint8_t address, uint8_t reg, uint8_t* data, std::size_t length) override;

    // Nine clock pulses with SDA released, which walks a slave that is holding
    // the line out of whatever transfer it thinks it is in. The standard
    // recovery and the only thing that gets a wedged bus back short of a
    // power cycle.
    void recover();

    uint32_t timeouts() const { return timeouts_; }

  private:
    uint32_t timeouts_{0};
};

}  // namespace pa::platform
