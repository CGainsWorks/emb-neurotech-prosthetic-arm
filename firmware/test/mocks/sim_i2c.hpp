// A simulated I2C bus with an LSM6DSOX on it.
//
// Enough of the device to exercise the driver end to end on a host: the
// register map, the WHO_AM_I, the data-ready bits, the burst read across the
// gyro and accelerometer blocks and the two-byte little-endian layout. It
// also models the failures the driver has to survive, because those are the
// paths that never get exercised on a bench and always get exercised in the
// field: a device that is not there, a bus that NACKs and a sensor that stops
// producing samples while still answering.
#pragma once

#include <cstring>
#include <vector>

#include "pa/hal/i2c_bus.hpp"
#include "pa/imu/lsm6dsox.hpp"

namespace sim {

class SimulatedI2cBus : public pa::hal::I2cBus {
  public:
    SimulatedI2cBus() { reset(); }

    void reset() {
        std::memset(registers_, 0, sizeof(registers_));
        registers_[pa::imu::reg::kWhoAmI] = pa::imu::kWhoAmIValue;
        present_ = true;
        fail_next_ = 0;
        data_ready_ = true;
        writes_.clear();
    }

    // --- test controls -----------------------------------------------------
    void set_present(bool present) { present_ = present; }
    void set_who_am_i(uint8_t value) { registers_[pa::imu::reg::kWhoAmI] = value; }
    void fail_next_transfers(int count) { fail_next_ = count; }
    void set_data_ready(bool ready) { data_ready_ = ready; }

    // Load a sample as the device would hold it: raw counts, little endian.
    void set_raw(const int16_t gyro[3], const int16_t accel[3], int16_t temperature) {
        for (int axis = 0; axis < 3; ++axis) {
            write_le(pa::imu::reg::kOutxLG + axis * 2, gyro[axis]);
            write_le(pa::imu::reg::kOutxLA + axis * 2, accel[axis]);
        }
        write_le(pa::imu::reg::kOutTempL, temperature);
    }

    uint8_t register_value(uint8_t reg) const { return registers_[reg]; }
    const std::vector<uint8_t>& written_registers() const { return writes_; }

    // --- bus ---------------------------------------------------------------
    pa::hal::I2cStatus write(uint8_t address, uint8_t reg, const uint8_t* data,
                             std::size_t length) override {
        if (!present_ || address != pa::imu::kDefaultAddress) return pa::hal::I2cStatus::kNack;
        if (fail_next_ > 0) {
            --fail_next_;
            return pa::hal::I2cStatus::kBusError;
        }
        for (std::size_t i = 0; i < length; ++i) {
            registers_[static_cast<uint8_t>(reg + i)] = data[i];
            writes_.push_back(static_cast<uint8_t>(reg + i));
        }
        return pa::hal::I2cStatus::kOk;
    }

    pa::hal::I2cStatus read(uint8_t address, uint8_t reg, uint8_t* data,
                            std::size_t length) override {
        if (!present_ || address != pa::imu::kDefaultAddress) return pa::hal::I2cStatus::kNack;
        if (fail_next_ > 0) {
            --fail_next_;
            return pa::hal::I2cStatus::kBusError;
        }
        if (reg == pa::imu::reg::kStatus) {
            // XLDA and GDA together, or neither.
            data[0] = data_ready_ ? 0x03 : 0x00;
            return pa::hal::I2cStatus::kOk;
        }
        for (std::size_t i = 0; i < length; ++i) data[i] = registers_[static_cast<uint8_t>(reg + i)];
        return pa::hal::I2cStatus::kOk;
    }

  private:
    void write_le(uint8_t reg, int16_t value) {
        const uint16_t bits = static_cast<uint16_t>(value);
        registers_[reg] = static_cast<uint8_t>(bits & 0xFF);
        registers_[static_cast<uint8_t>(reg + 1)] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    }

    uint8_t registers_[256]{};
    bool present_{true};
    int fail_next_{0};
    bool data_ready_{true};
    std::vector<uint8_t> writes_;
};

}  // namespace sim
