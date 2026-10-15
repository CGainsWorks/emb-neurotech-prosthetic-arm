// Servo output: joint angles in, pulse widths out, with the limits enforced
// here rather than trusted to whatever sent the command.
//
// The last stage before a limb moves. Everything upstream of it, host
// included, is treated as capable of sending nonsense: a NaN, a step from one
// stop to the other, a command that arrived a second ago. This file is where
// those stop.
#pragma once

#include <array>
#include <cstdint>

#include "pa/config/board.hpp"

namespace pa::servo {

// Per-channel calibration. `trim_us` and `reversed` are the two things that
// always differ between a drawing and an assembled arm, so they are data
// rather than constants and are stored in flash at bring-up.
struct ChannelCalibration {
    float trim_us{0.0f};             // added to the pulse, corrects horn splines
    float gear_ratio{1.0f};          // joint radians per servo radian
    bool reversed{false};            // the horn turns the other way to the joint
    float lower_rad{-1.5f};          // mechanical stop, joint side
    float upper_rad{1.5f};           // mechanical stop, joint side
    float max_rate_rad_s{4.0f};      // slew limit, joint side
    // Stall current at the bus voltage, from the datasheet. Used for the
    // current estimate below and by the thermal budget.
    float stall_current_a{2.5f};
};

struct ChannelState {
    float commanded_rad{0.0f};   // after limiting, what the servo was asked for
    float pulse_us{config::kServoCentrePulseUs};
    bool clamped_by_stop{false};
    bool clamped_by_rate{false};
    bool rejected{false};        // the command was not a finite number
};

// Angle to pulse width for one channel, with the stops and the slew limit
// applied. Kept separate from the six-channel driver so it can be tested on
// its own and so the conversion is usable by the bring-up tools.
class Servo {
  public:
    Servo() = default;
    explicit Servo(const ChannelCalibration& calibration) : calibration_(calibration) {}

    void configure(const ChannelCalibration& calibration);

    // Ask for `target_rad` given that `dt` has passed. Returns the state after
    // limiting; the pulse width is always inside the electrical range whatever
    // was asked for.
    ChannelState update(float target_rad, float dt);

    // Hold the present angle. What a command timeout produces: the arm stops
    // where it is rather than falling to a default pose.
    ChannelState hold();

    float angle() const { return angle_rad_; }
    void set_angle(float radians) { angle_rad_ = radians; }

    const ChannelCalibration& calibration() const { return calibration_; }

    // Pulse width for an angle, ignoring the slew limit. The inverse of
    // pulse_to_angle and tested as such.
    float angle_to_pulse(float radians) const;
    float pulse_to_angle(float pulse_us) const;

  private:
    ChannelCalibration calibration_{};
    float angle_rad_{0.0f};
};

struct DriveState {
    std::array<ChannelState, config::kServoChannels> channels{};
    float estimated_current_a{0.0f};
    bool any_rejected{false};
    bool any_clamped{false};
};

// All six channels, plus the current estimate the safety layer trips on.
class Driver {
  public:
    Driver();

    void configure(std::size_t index, const ChannelCalibration& calibration);
    void set_angles(const std::array<float, config::kServoChannels>& radians);

    DriveState update(const std::array<float, config::kServoChannels>& targets, float dt);
    DriveState hold();

    // Outputs disabled: pulses stop and a servo with no pulse train goes
    // limp rather than holding. Deliberate. A powered arm that has lost its
    // host should not keep pushing against whatever it is touching.
    DriveState disable();

    std::array<float, config::kServoChannels> angles() const;
    const Servo& channel(std::size_t index) const { return channels_[index]; }
    Servo& channel(std::size_t index) { return channels_[index]; }

    bool enabled() const { return enabled_; }

  private:
    std::array<Servo, config::kServoChannels> channels_{};
    std::array<float, config::kServoChannels> previous_targets_{};
    bool enabled_{true};
};

// Rough bus current from how hard each channel is being driven.
//
// This is an estimate from the commanded step, not a measurement: the board
// has a shunt and an INA219 on the drive rail for the real thing and nothing
// has ever been measured because no board exists. The estimate is what the
// safety layer trips on until the shunt reading is available and it is
// deliberately pessimistic.
float estimate_current(const std::array<ChannelState, config::kServoChannels>& channels,
                       const std::array<ChannelCalibration, config::kServoChannels>& calibration,
                       const std::array<float, config::kServoChannels>& previous_targets);

}  // namespace pa::servo
