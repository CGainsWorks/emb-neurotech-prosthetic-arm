#include "pa/servo/driver.hpp"

#include <algorithm>
#include <cmath>

namespace pa::servo {
namespace {

float clamp(float value, float low, float high) {
    return std::max(low, std::min(high, value));
}

}  // namespace

void Servo::configure(const ChannelCalibration& calibration) {
    calibration_ = calibration;
    angle_rad_ = clamp(angle_rad_, calibration_.lower_rad, calibration_.upper_rad);
}

float Servo::angle_to_pulse(float radians) const {
    const float servo_rad = calibration_.gear_ratio != 0.0f ? radians / calibration_.gear_ratio
                                                            : radians;
    const float signed_rad = calibration_.reversed ? -servo_rad : servo_rad;
    const float pulse = config::kServoCentrePulseUs + signed_rad * config::kServoUsPerRadian +
                        calibration_.trim_us;
    return clamp(pulse, config::kServoMinPulseUs, config::kServoMaxPulseUs);
}

float Servo::pulse_to_angle(float pulse_us) const {
    const float signed_rad =
        (pulse_us - config::kServoCentrePulseUs - calibration_.trim_us) / config::kServoUsPerRadian;
    const float servo_rad = calibration_.reversed ? -signed_rad : signed_rad;
    return servo_rad * calibration_.gear_ratio;
}

ChannelState Servo::update(float target_rad, float dt) {
    ChannelState state;

    // A non-finite target is the one input that cannot be clamped into
    // something sensible, so it is refused outright and the channel holds.
    if (!std::isfinite(target_rad) || !std::isfinite(dt) || dt <= 0.0f) {
        state.rejected = true;
        state.commanded_rad = angle_rad_;
        state.pulse_us = angle_to_pulse(angle_rad_);
        return state;
    }

    const float stopped = clamp(target_rad, calibration_.lower_rad, calibration_.upper_rad);
    state.clamped_by_stop = stopped != target_rad;

    const float max_step = calibration_.max_rate_rad_s * dt;
    const float requested_step = stopped - angle_rad_;
    const float limited_step = clamp(requested_step, -max_step, max_step);
    state.clamped_by_rate = limited_step != requested_step;

    angle_rad_ += limited_step;
    state.commanded_rad = angle_rad_;
    state.pulse_us = angle_to_pulse(angle_rad_);
    return state;
}

ChannelState Servo::hold() {
    ChannelState state;
    state.commanded_rad = angle_rad_;
    state.pulse_us = angle_to_pulse(angle_rad_);
    return state;
}

Driver::Driver() {
    for (auto& channel : channels_) channel.configure(ChannelCalibration{});
}

void Driver::configure(std::size_t index, const ChannelCalibration& calibration) {
    if (index >= config::kServoChannels) return;
    channels_[index].configure(calibration);
}

void Driver::set_angles(const std::array<float, config::kServoChannels>& radians) {
    for (std::size_t i = 0; i < config::kServoChannels; ++i) channels_[i].set_angle(radians[i]);
}

DriveState Driver::update(const std::array<float, config::kServoChannels>& targets, float dt) {
    if (!enabled_) return disable();

    DriveState state;
    std::array<ChannelCalibration, config::kServoChannels> calibration{};
    for (std::size_t i = 0; i < config::kServoChannels; ++i) {
        state.channels[i] = channels_[i].update(targets[i], dt);
        calibration[i] = channels_[i].calibration();
        state.any_rejected = state.any_rejected || state.channels[i].rejected;
        state.any_clamped = state.any_clamped || state.channels[i].clamped_by_stop ||
                            state.channels[i].clamped_by_rate;
    }
    state.estimated_current_a = estimate_current(state.channels, calibration, previous_targets_);
    for (std::size_t i = 0; i < config::kServoChannels; ++i)
        previous_targets_[i] = state.channels[i].commanded_rad;
    return state;
}

DriveState Driver::hold() {
    DriveState state;
    std::array<ChannelCalibration, config::kServoChannels> calibration{};
    for (std::size_t i = 0; i < config::kServoChannels; ++i) {
        state.channels[i] = channels_[i].hold();
        calibration[i] = channels_[i].calibration();
    }
    state.estimated_current_a = estimate_current(state.channels, calibration, previous_targets_);
    for (std::size_t i = 0; i < config::kServoChannels; ++i)
        previous_targets_[i] = state.channels[i].commanded_rad;
    return state;
}

DriveState Driver::disable() {
    enabled_ = false;
    DriveState state;
    for (std::size_t i = 0; i < config::kServoChannels; ++i) {
        // No pulse. Recorded as the centre value so the telemetry has
        // something to show but the timer channel is off; see
        // src/platform/stm32/pwm_servo.cpp.
        state.channels[i].commanded_rad = channels_[i].angle();
        state.channels[i].pulse_us = 0.0f;
    }
    state.estimated_current_a = 0.0f;
    return state;
}

std::array<float, config::kServoChannels> Driver::angles() const {
    std::array<float, config::kServoChannels> out{};
    for (std::size_t i = 0; i < config::kServoChannels; ++i) out[i] = channels_[i].angle();
    return out;
}

float estimate_current(const std::array<ChannelState, config::kServoChannels>& channels,
                       const std::array<ChannelCalibration, config::kServoChannels>& calibration,
                       const std::array<float, config::kServoChannels>& previous_targets) {
    // A hobby servo's current is dominated by how far it is from where it has
    // been told to be. Quiescent draw is a fixed fraction of stall; the rest
    // scales with the tracking error, saturating at stall.
    //
    // Deliberately pessimistic. The alternative to over-estimating here is a
    // fuse that opens or a regulator that folds back, both of which drop a
    // limb.
    constexpr float kQuiescentFraction = 0.06f;
    constexpr float kAmpsPerRadianError = 4.0f;

    float total = 0.0f;
    for (std::size_t i = 0; i < config::kServoChannels; ++i) {
        if (channels[i].pulse_us <= 0.0f) continue;  // output disabled, no draw
        const float error = std::fabs(channels[i].commanded_rad - previous_targets[i]);
        const float dynamic = std::min(1.0f, error * kAmpsPerRadianError);
        total += calibration[i].stall_current_a * (kQuiescentFraction + dynamic);
    }
    return total;
}

}  // namespace pa::servo
