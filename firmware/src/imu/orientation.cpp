#include "pa/imu/orientation.hpp"

#include <cmath>

namespace pa::imu {
namespace {

// How far the measured acceleration may be from one g before the sample is
// treated as containing arm movement rather than only gravity.
constexpr float kStationaryToleranceMps2 = 0.35f;
// And how slowly the arm has to be turning at the same time.
constexpr float kStationaryRateRadS = 0.05f;

// The estimate is not published until the filter has run for a couple of time
// constants, so a start-up transient is never mistaken for an arm position.
constexpr float kSettleTimeConstants = 2.0f;

}  // namespace

void accelerometer_angles(const float accel_mps2[3], float& roll_rad, float& pitch_rad) {
    const float x = accel_mps2[0], y = accel_mps2[1], z = accel_mps2[2];
    roll_rad = std::atan2(y, z);
    // atan2 against the magnitude of the other two rather than against z
    // alone, so pitch stays continuous through vertical instead of jumping by
    // pi when z changes sign.
    pitch_rad = std::atan2(-x, std::sqrt(y * y + z * z));
}

void ComplementaryFilter::reset() {
    state_ = Orientation{};
    settled_s_ = 0.0f;
    dropout_s_ = 0.0f;
    stationary_ = false;
}

Orientation ComplementaryFilter::update(const Sample& sample, float dt) {
    if (!(dt > 0.0f) || !std::isfinite(dt)) return state_;

    const float magnitude = std::sqrt(sample.accel_mps2[0] * sample.accel_mps2[0] +
                                      sample.accel_mps2[1] * sample.accel_mps2[1] +
                                      sample.accel_mps2[2] * sample.accel_mps2[2]);
    const float rate = std::sqrt(sample.gyro_rad_s[0] * sample.gyro_rad_s[0] +
                                 sample.gyro_rad_s[1] * sample.gyro_rad_s[1] +
                                 sample.gyro_rad_s[2] * sample.gyro_rad_s[2]);
    stationary_ = std::fabs(magnitude - kGravity) < kStationaryToleranceMps2 &&
                  rate < kStationaryRateRadS;

    // Gyro first: integrate the rates forward.
    float roll = state_.roll_rad + sample.gyro_rad_s[0] * dt;
    float pitch = state_.pitch_rad + sample.gyro_rad_s[1] * dt;

    // Then pull towards the accelerometer but only when it is plausibly
    // measuring gravity. During a fast movement the accelerometer reads the
    // arm's own acceleration and blending it in would drag the estimate off.
    if (magnitude > 0.5f * kGravity && magnitude < 1.5f * kGravity) {
        float accel_roll = 0.0f, accel_pitch = 0.0f;
        accelerometer_angles(sample.accel_mps2, accel_roll, accel_pitch);

        const float alpha = time_constant_s_ / (time_constant_s_ + dt);
        roll = alpha * roll + (1.0f - alpha) * accel_roll;
        pitch = alpha * pitch + (1.0f - alpha) * accel_pitch;
    }

    state_.roll_rad = roll;
    state_.pitch_rad = pitch;

    dropout_s_ = 0.0f;
    settled_s_ += dt;
    state_.trusted = settled_s_ >= kSettleTimeConstants * time_constant_s_;
    return state_;
}

Orientation ComplementaryFilter::coast(float dt) {
    if (!(dt > 0.0f) || !std::isfinite(dt)) return state_;
    dropout_s_ += dt;
    stationary_ = false;
    // Without a sample there is nothing to integrate, so the angles are held.
    // Past one time constant the gyro drift alone would have moved them by
    // more than the filter could correct, so the estimate stops being trusted.
    if (dropout_s_ > time_constant_s_) state_.trusted = false;
    return state_;
}

}  // namespace pa::imu
