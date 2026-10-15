// LSM6DSOX inertial measurement unit, over I2C.
//
// Chosen over the more common six-axis parts for one reason: a flat register
// map. The ICM-20948 banks its registers, so every configuration write costs a
// bank select and a driver that has to track which bank it left the device in
// is a driver with a state machine nobody tests. This part has none of that.
//
// The IMU is not in the control loop. It provides the orientation of the
// forearm relative to gravity, which is what tells the arm whether it is being
// held out in front of the wearer or hanging by their side and the gesture
// mapping uses that to choose between grasp poses. If it drops out, the arm
// keeps working with the last known orientation and raises a fault.
#pragma once

#include <cstdint>

#include "pa/hal/i2c_bus.hpp"

namespace pa::imu {

// Register addresses, LSM6DSOX datasheet section 9.
namespace reg {
inline constexpr uint8_t kWhoAmI = 0x0F;
inline constexpr uint8_t kCtrl1Xl = 0x10;
inline constexpr uint8_t kCtrl2G = 0x11;
inline constexpr uint8_t kCtrl3C = 0x12;
inline constexpr uint8_t kCtrl6C = 0x15;
inline constexpr uint8_t kStatus = 0x1E;
inline constexpr uint8_t kOutTempL = 0x20;
inline constexpr uint8_t kOutxLG = 0x22;   // gyro, six bytes
inline constexpr uint8_t kOutxLA = 0x28;   // accel, six bytes
}  // namespace reg

inline constexpr uint8_t kWhoAmIValue = 0x6C;
inline constexpr uint8_t kDefaultAddress = 0x6A;  // SDO tied low

struct Sample {
    float accel_mps2[3]{};
    float gyro_rad_s[3]{};
    float temperature_c{0.0f};
    uint32_t timestamp_us{0};
};

enum class ImuStatus : uint8_t {
    kOk = 0,
    kNotPresent,     // WHO_AM_I did not read back the expected value
    kBusFault,       // the transfer itself failed
    kNoNewData,      // the device has not produced a sample since the last read
};

class Lsm6dsox {
  public:
    Lsm6dsox(hal::I2cBus& bus, uint8_t address = kDefaultAddress) : bus_(bus), address_(address) {}

    // Reset, identify and configure for 208 Hz on both sensors at plus or
    // minus 4 g and 500 degrees per second.
    //
    // 208 Hz rather than the 104 Hz that would just satisfy the 100 Hz the
    // orientation filter runs at: the next rate down leaves no margin for the
    // sample being read a little late and this part has no intermediate step.
    ImuStatus begin();

    // Read one sample. Returns kNoNewData rather than the previous values when
    // the device has not updated, so a stalled sensor cannot be mistaken for a
    // stationary arm.
    ImuStatus read(Sample& sample, uint32_t timestamp_us);

    // Consecutive failed reads. The fault layer trips on this rather than on a
    // single failure, because one NACK on a shared bus is normal.
    uint32_t consecutive_failures() const { return consecutive_failures_; }

    bool present() const { return present_; }

    // Zero-rate offset, learned while the arm is known to be still. Gyro bias
    // is the whole reason a complementary filter is needed rather than plain
    // integration.
    void set_gyro_bias(const float bias_rad_s[3]);
    const float* gyro_bias() const { return gyro_bias_; }

  private:
    hal::I2cBus& bus_;
    uint8_t address_;
    bool present_{false};
    uint32_t consecutive_failures_{0};
    float gyro_bias_[3]{0.0f, 0.0f, 0.0f};
};

// Datasheet sensitivities for the ranges begin() selects.
inline constexpr float kAccelSensitivityG = 0.122e-3f;      // g per count at plus or minus 4 g
inline constexpr float kGyroSensitivityDps = 17.50e-3f;     // dps per count at plus or minus 500
inline constexpr float kGravity = 9.80665f;

}  // namespace pa::imu
