#include <cmath>
#include <limits>

#include "pa/servo/driver.hpp"
#include "test_framework.hpp"

using namespace pa;
using namespace pa::servo;

namespace {

ChannelCalibration plain() {
    ChannelCalibration calibration;
    calibration.lower_rad = -1.5f;
    calibration.upper_rad = 1.5f;
    calibration.max_rate_rad_s = 4.0f;
    return calibration;
}

}  // namespace

TEST(centre_angle_is_centre_pulse) {
    Servo servo(plain());
    CHECK_NEAR(servo.angle_to_pulse(0.0f), config::kServoCentrePulseUs, 1e-3);
}

TEST(angle_and_pulse_are_inverses) {
    ChannelCalibration calibration = plain();
    calibration.trim_us = 37.0f;
    calibration.gear_ratio = 1.8f;
    calibration.reversed = true;
    Servo servo(calibration);

    for (float angle = -1.4f; angle <= 1.4f; angle += 0.05f) {
        const float pulse = servo.angle_to_pulse(angle);
        CHECK_NEAR(servo.pulse_to_angle(pulse), angle, 1e-4);
    }
}

TEST(a_pulse_never_leaves_the_electrical_range) {
    // Whatever is asked for. A servo given a pulse outside its range either
    // ignores it or drives to a stop and stalls and the second one burns a
    // motor.
    Servo servo(plain());
    for (float angle = -100.0f; angle <= 100.0f; angle += 0.37f) {
        const float pulse = servo.angle_to_pulse(angle);
        CHECK(pulse >= config::kServoMinPulseUs);
        CHECK(pulse <= config::kServoMaxPulseUs);
    }
}

TEST(the_mechanical_stops_are_enforced_here_not_upstream) {
    Servo servo(plain());
    const ChannelState high = servo.update(3.0f, 10.0f);
    CHECK(high.clamped_by_stop);
    CHECK_NEAR(high.commanded_rad, 1.5f, 1e-6);

    const ChannelState low = servo.update(-3.0f, 10.0f);
    CHECK(low.clamped_by_stop);
    CHECK_NEAR(low.commanded_rad, -1.5f, 1e-6);
}

TEST(the_slew_limit_turns_a_step_into_a_ramp) {
    Servo servo(plain());
    constexpr float kDt = 0.003f;  // one servo frame
    const ChannelState first = servo.update(1.4f, kDt);
    CHECK(first.clamped_by_rate);
    CHECK_NEAR(first.commanded_rad, 4.0f * kDt, 1e-6);

    // And it arrives eventually, at the rate it was told.
    int frames = 1;
    while (std::fabs(servo.angle() - 1.4f) > 1e-4f && frames < 10000) {
        servo.update(1.4f, kDt);
        ++frames;
    }
    CHECK_NEAR(static_cast<float>(frames) * kDt, 1.4f / 4.0f, 0.01);
}

TEST(a_command_that_is_not_a_number_is_refused) {
    // The one input that cannot be clamped into something sensible. A NaN
    // through the clamp would come out the other side unchanged and be handed
    // to a timer.
    Servo servo(plain());
    servo.update(0.5f, 1.0f);
    const float before = servo.angle();

    for (float bad : {std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()}) {
        const ChannelState state = servo.update(bad, 0.003f);
        CHECK(state.rejected);
        CHECK_NEAR(servo.angle(), before, 1e-9);
        CHECK(std::isfinite(state.pulse_us));
        CHECK(state.pulse_us >= config::kServoMinPulseUs);
    }
}

TEST(a_nonsense_timestep_is_refused_too) {
    Servo servo(plain());
    servo.update(0.5f, 1.0f);
    const float before = servo.angle();
    CHECK(servo.update(1.0f, 0.0f).rejected);
    CHECK(servo.update(1.0f, -0.01f).rejected);
    CHECK(servo.update(1.0f, std::numeric_limits<float>::quiet_NaN()).rejected);
    CHECK_NEAR(servo.angle(), before, 1e-9);
}

TEST(a_reversed_channel_moves_the_other_way) {
    ChannelCalibration forward = plain();
    ChannelCalibration backward = plain();
    backward.reversed = true;

    Servo a(forward), b(backward);
    const float pulse_a = a.angle_to_pulse(0.5f);
    const float pulse_b = b.angle_to_pulse(0.5f);
    CHECK(pulse_a > config::kServoCentrePulseUs);
    CHECK(pulse_b < config::kServoCentrePulseUs);
    CHECK_NEAR(pulse_a - config::kServoCentrePulseUs,
               config::kServoCentrePulseUs - pulse_b, 1e-3);
}

TEST(the_gear_ratio_scales_the_joint_against_the_horn) {
    ChannelCalibration geared = plain();
    geared.gear_ratio = 3.0f;  // three joint radians per servo radian
    Servo servo(geared);

    // A joint angle three times larger produces the same horn angle, so the
    // same pulse.
    Servo direct(plain());
    CHECK_NEAR(servo.angle_to_pulse(0.9f), direct.angle_to_pulse(0.3f), 1e-3);
}

TEST(holding_keeps_the_angle_and_the_pulse) {
    Servo servo(plain());
    servo.update(0.7f, 10.0f);
    const ChannelState held = servo.hold();
    CHECK_NEAR(held.commanded_rad, 0.7f, 1e-6);
    CHECK_NEAR(held.pulse_us, servo.angle_to_pulse(0.7f), 1e-6);
    CHECK(!held.clamped_by_rate);
}

TEST(the_driver_moves_all_six_channels_and_reports_what_it_clamped) {
    Driver driver;
    for (std::size_t i = 0; i < config::kServoChannels; ++i) driver.configure(i, plain());

    std::array<float, config::kServoChannels> targets{};
    targets.fill(0.1f);
    targets[2] = 99.0f;  // past the stop

    const DriveState state = driver.update(targets, 1.0f);
    CHECK(state.any_clamped);
    CHECK(state.channels[2].clamped_by_stop);
    CHECK(!state.channels[0].clamped_by_stop);
    CHECK(!state.any_rejected);
}

TEST(disabling_the_driver_stops_the_pulses) {
    // A servo with no pulse train relaxes. Deliberate: an arm that has lost
    // its host should stop pushing against whatever it is touching, not hold
    // whatever it was doing indefinitely.
    Driver driver;
    std::array<float, config::kServoChannels> targets{};
    targets.fill(0.4f);
    driver.update(targets, 1.0f);

    const DriveState off = driver.disable();
    CHECK(!driver.enabled());
    for (const auto& channel : off.channels) CHECK_NEAR(channel.pulse_us, 0.0f, 1e-9);
    CHECK_NEAR(off.estimated_current_a, 0.0f, 1e-9);
}

TEST(the_current_estimate_rises_with_how_hard_the_arm_is_driven) {
    Driver still, moving;
    std::array<float, config::kServoChannels> hold{};
    std::array<float, config::kServoChannels> sweep{};
    sweep.fill(1.0f);

    // Settle both at zero.
    still.update(hold, 1.0f);
    moving.update(hold, 1.0f);

    const float quiescent = still.update(hold, 0.003f).estimated_current_a;
    const float driven = moving.update(sweep, 0.003f).estimated_current_a;

    CHECK(quiescent > 0.0f);
    CHECK(driven > quiescent);
    // And it saturates rather than growing without bound, because a servo
    // cannot draw more than its stall current.
    std::array<float, config::kServoChannels> far{};
    far.fill(50.0f);
    const float stalled = moving.update(far, 1.0f).estimated_current_a;
    CHECK(stalled <= 6.0f * plain().stall_current_a * 1.07f);
}

namespace {

// The arm as it is actually built: the three proximal joints carry the large
// actuators. Matches calibration_for() in firmware/src/main.cpp.
Driver arm_driver() {
    Driver driver;
    for (std::size_t i = 0; i < config::kServoChannels; ++i) {
        ChannelCalibration calibration = plain();
        calibration.stall_current_a = i < 3 ? 2.5f : 1.2f;
        driver.configure(i, calibration);
    }
    return driver;
}

}  // namespace

TEST(a_synchronised_move_stays_under_the_current_trip) {
    // Every joint moving at once, at its rate limit, updated at the servo
    // frame rate. This is the worst case the planner can produce, because it
    // synchronises the joints on purpose and it has to sit below the trip or
    // the arm would fault on an ordinary movement.
    Driver driver = arm_driver();
    const float dt = 1.0f / config::kServoFrameRateHz;

    std::array<float, config::kServoChannels> targets{};
    targets.fill(1.4f);

    float worst = 0.0f;
    for (int frame = 0; frame < 400; ++frame) {
        const DriveState state = driver.update(targets, dt);
        worst = std::max(worst, state.estimated_current_a);
    }
    CHECK(worst < config::kBusCurrentLimitAmps);
    CHECK(worst > 0.0f);
}

TEST(a_jam_on_every_joint_exceeds_the_current_trip) {
    // The other half of the same claim. A trip that no fault can reach is not
    // a protection and the 8 A figure was chosen to sit between the two.
    Driver driver = arm_driver();

    // A step far larger than a frame's worth of movement is what a servo
    // pushing against a jam looks like to this estimator.
    std::array<float, config::kServoChannels> targets{};
    targets.fill(1.4f);
    const DriveState state = driver.update(targets, 1.0f);

    CHECK(state.estimated_current_a > config::kBusCurrentLimitAmps);
    // And not more than every actuator can physically draw.
    CHECK(state.estimated_current_a <= (3.0f * 2.5f + 3.0f * 1.2f) * 1.07f);
}

TEST_MAIN()
