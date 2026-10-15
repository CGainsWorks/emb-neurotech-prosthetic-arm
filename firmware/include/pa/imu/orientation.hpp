// Roll and pitch of the forearm from the IMU, by complementary filter.
//
// A complementary filter rather than a Kalman one, on purpose. There is no
// magnetometer on this board, so yaw is unobservable and no amount of
// estimator sophistication will recover it; what is left is two angles from
// two sensors with complementary error spectra, which is the textbook case
// where the simple filter gives up nothing. It also runs in a few dozen flops
// and has one tuning parameter with a physical meaning.
#pragma once

#include <cstdint>

#include "pa/imu/lsm6dsox.hpp"

namespace pa::imu {

struct Orientation {
    float roll_rad{0.0f};
    float pitch_rad{0.0f};
    bool trusted{false};  // false until the filter has settled or after a dropout
};

class ComplementaryFilter {
  public:
    // `time_constant_s` is the crossover: below it the gyro is believed,
    // above it the accelerometer is. Half a second puts the crossover well
    // below the frequencies of arm movement and well above the gyro's drift.
    explicit ComplementaryFilter(float time_constant_s = 0.5f)
        : time_constant_s_(time_constant_s) {}

    void reset();

    // Feed one IMU sample. `dt` is the interval since the previous call.
    Orientation update(const Sample& sample, float dt);

    // Told there is no new sample this tick: propagate on the gyro alone and
    // stop trusting the estimate once the dropout is longer than the time
    // constant.
    Orientation coast(float dt);

    Orientation state() const { return state_; }

    // True while the arm is still enough for the accelerometer to be measuring
    // gravity and nothing else. The gyro bias estimate is only updated here.
    bool stationary() const { return stationary_; }

  private:
    float time_constant_s_;
    Orientation state_{};
    float settled_s_{0.0f};
    float dropout_s_{0.0f};
    bool stationary_{false};
};

// Roll and pitch from the accelerometer alone, assuming it is measuring
// nothing but gravity. Exact when the arm is still and wrong the instant it
// is not, which is the whole reason the gyro is there.
void accelerometer_angles(const float accel_mps2[3], float& roll_rad, float& pitch_rad);

}  // namespace pa::imu
