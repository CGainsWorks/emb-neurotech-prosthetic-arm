#include "pa/imu/lsm6dsox.hpp"

namespace pa::imu {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

int16_t little_endian(const uint8_t* bytes) {
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                                (static_cast<uint16_t>(bytes[1]) << 8));
}

}  // namespace

ImuStatus Lsm6dsox::begin() {
    present_ = false;

    uint8_t who = 0;
    if (bus_.read_byte(address_, reg::kWhoAmI, who) != hal::I2cStatus::kOk) return ImuStatus::kBusFault;
    if (who != kWhoAmIValue) return ImuStatus::kNotPresent;

    // BDU set, so the high and low halves of a reading always belong to the
    // same sample; IF_INC set so a six-byte burst walks the registers. Without
    // BDU a burst read across an update boundary silently returns half of one
    // sample and half of the next, which reads as a plausible spike.
    constexpr uint8_t kBduAndAutoIncrement = 0x44;
    if (bus_.write_byte(address_, reg::kCtrl3C, kBduAndAutoIncrement) != hal::I2cStatus::kOk)
        return ImuStatus::kBusFault;

    // 208 Hz, plus or minus 4 g.
    constexpr uint8_t kAccel208Hz4g = 0x58;
    if (bus_.write_byte(address_, reg::kCtrl1Xl, kAccel208Hz4g) != hal::I2cStatus::kOk)
        return ImuStatus::kBusFault;

    // 208 Hz, plus or minus 500 dps.
    constexpr uint8_t kGyro208Hz500dps = 0x54;
    if (bus_.write_byte(address_, reg::kCtrl2G, kGyro208Hz500dps) != hal::I2cStatus::kOk)
        return ImuStatus::kBusFault;

    present_ = true;
    consecutive_failures_ = 0;
    return ImuStatus::kOk;
}

ImuStatus Lsm6dsox::read(Sample& sample, uint32_t timestamp_us) {
    if (!present_) return ImuStatus::kNotPresent;

    uint8_t status = 0;
    if (bus_.read_byte(address_, reg::kStatus, status) != hal::I2cStatus::kOk) {
        ++consecutive_failures_;
        return ImuStatus::kBusFault;
    }
    // XLDA and GDA: new accelerometer and gyroscope data respectively.
    constexpr uint8_t kBothReady = 0x03;
    if ((status & kBothReady) != kBothReady) return ImuStatus::kNoNewData;

    uint8_t raw[12]{};
    if (bus_.read(address_, reg::kOutxLG, raw, sizeof(raw)) != hal::I2cStatus::kOk) {
        ++consecutive_failures_;
        return ImuStatus::kBusFault;
    }

    uint8_t temperature_bytes[2]{};
    if (bus_.read(address_, reg::kOutTempL, temperature_bytes, sizeof(temperature_bytes)) !=
        hal::I2cStatus::kOk) {
        ++consecutive_failures_;
        return ImuStatus::kBusFault;
    }

    for (int axis = 0; axis < 3; ++axis) {
        const int16_t gyro = little_endian(raw + axis * 2);
        const int16_t accel = little_endian(raw + 6 + axis * 2);
        sample.gyro_rad_s[axis] =
            static_cast<float>(gyro) * kGyroSensitivityDps * kDegToRad - gyro_bias_[axis];
        sample.accel_mps2[axis] = static_cast<float>(accel) * kAccelSensitivityG * kGravity;
    }

    // 256 counts per degree, zero at 25 C.
    sample.temperature_c = static_cast<float>(little_endian(temperature_bytes)) / 256.0f + 25.0f;
    sample.timestamp_us = timestamp_us;

    consecutive_failures_ = 0;
    return ImuStatus::kOk;
}

void Lsm6dsox::set_gyro_bias(const float bias_rad_s[3]) {
    for (int axis = 0; axis < 3; ++axis) gyro_bias_[axis] = bias_rad_s[axis];
}

}  // namespace pa::imu
