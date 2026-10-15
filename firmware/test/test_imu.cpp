#include <cmath>

#include "mocks/sim_i2c.hpp"
#include "pa/imu/lsm6dsox.hpp"
#include "pa/imu/orientation.hpp"
#include "test_framework.hpp"

using namespace pa::imu;

namespace {

constexpr float kPi = 3.14159265358979323846f;

int16_t accel_counts(float mps2) {
    return static_cast<int16_t>(mps2 / (kAccelSensitivityG * kGravity));
}

int16_t gyro_counts(float rad_s) {
    const float dps = rad_s * 180.0f / kPi;
    return static_cast<int16_t>(dps / kGyroSensitivityDps);
}

}  // namespace

TEST(the_driver_identifies_the_part_before_configuring_it) {
    sim::SimulatedI2cBus bus;
    Lsm6dsox imu(bus);
    CHECK(imu.begin() == ImuStatus::kOk);
    CHECK(imu.present());

    // And refuses to configure something that is not the part it expects.
    // Writing accelerometer settings into whatever answered at that address is
    // how a wrong-part bring-up turns into a mystery.
    bus.reset();
    bus.set_who_am_i(0x68);
    Lsm6dsox wrong(bus);
    CHECK(wrong.begin() == ImuStatus::kNotPresent);
    CHECK(!wrong.present());
    CHECK(bus.written_registers().empty());
}

TEST(the_driver_reports_an_absent_device_rather_than_hanging) {
    sim::SimulatedI2cBus bus;
    bus.set_present(false);
    Lsm6dsox imu(bus);
    CHECK(imu.begin() == ImuStatus::kBusFault);
    CHECK(!imu.present());

    Sample sample;
    CHECK(imu.read(sample, 0) == ImuStatus::kNotPresent);
}

TEST(configuration_sets_block_data_update_and_auto_increment) {
    // Without BDU a burst read that straddles a sensor update returns the low
    // half of one sample and the high half of the next, which looks like a
    // plausible spike rather than like an error.
    sim::SimulatedI2cBus bus;
    Lsm6dsox imu(bus);
    CHECK(imu.begin() == ImuStatus::kOk);

    const uint8_t ctrl3 = bus.register_value(reg::kCtrl3C);
    CHECK((ctrl3 & 0x40) != 0);  // BDU
    CHECK((ctrl3 & 0x04) != 0);  // IF_INC
    CHECK(bus.register_value(reg::kCtrl1Xl) != 0);
    CHECK(bus.register_value(reg::kCtrl2G) != 0);
}

TEST(a_sample_converts_to_metres_and_radians) {
    sim::SimulatedI2cBus bus;
    Lsm6dsox imu(bus);
    CHECK(imu.begin() == ImuStatus::kOk);

    const int16_t gyro[3] = {gyro_counts(1.0f), gyro_counts(-2.0f), gyro_counts(0.5f)};
    const int16_t accel[3] = {accel_counts(0.0f), accel_counts(0.0f), accel_counts(kGravity)};
    bus.set_raw(gyro, accel, static_cast<int16_t>(256 * 10));  // 35 C

    Sample sample;
    CHECK(imu.read(sample, 1234) == ImuStatus::kOk);
    CHECK_NEAR(sample.gyro_rad_s[0], 1.0f, 0.01);
    CHECK_NEAR(sample.gyro_rad_s[1], -2.0f, 0.01);
    CHECK_NEAR(sample.gyro_rad_s[2], 0.5f, 0.01);
    CHECK_NEAR(sample.accel_mps2[2], kGravity, 0.02);
    CHECK_NEAR(sample.temperature_c, 35.0f, 0.1);
    CHECK_EQ(static_cast<int>(sample.timestamp_us), 1234);
}

TEST(a_stalled_sensor_is_not_reported_as_a_stationary_arm) {
    // The failure that matters: the device answers, the registers still hold
    // the last sample and without the data-ready check the arm looks perfectly
    // still rather than broken.
    sim::SimulatedI2cBus bus;
    Lsm6dsox imu(bus);
    CHECK(imu.begin() == ImuStatus::kOk);

    Sample sample;
    CHECK(imu.read(sample, 0) == ImuStatus::kOk);

    bus.set_data_ready(false);
    CHECK(imu.read(sample, 1) == ImuStatus::kNoNewData);
}

TEST(consecutive_bus_failures_are_counted) {
    sim::SimulatedI2cBus bus;
    Lsm6dsox imu(bus);
    CHECK(imu.begin() == ImuStatus::kOk);

    Sample sample;
    bus.fail_next_transfers(3);
    for (int i = 0; i < 3; ++i) CHECK(imu.read(sample, 0) == ImuStatus::kBusFault);
    CHECK_EQ(static_cast<int>(imu.consecutive_failures()), 3);

    // One good read clears it: a single NACK on a shared bus is normal and
    // tripping a fault on one would be a fault every few minutes.
    CHECK(imu.read(sample, 0) == ImuStatus::kOk);
    CHECK_EQ(static_cast<int>(imu.consecutive_failures()), 0);
}

TEST(the_gyro_bias_is_subtracted) {
    sim::SimulatedI2cBus bus;
    Lsm6dsox imu(bus);
    CHECK(imu.begin() == ImuStatus::kOk);

    const int16_t gyro[3] = {gyro_counts(0.03f), 0, 0};
    const int16_t accel[3] = {0, 0, accel_counts(kGravity)};
    bus.set_raw(gyro, accel, 0);

    Sample uncorrected;
    CHECK(imu.read(uncorrected, 0) == ImuStatus::kOk);
    CHECK(std::fabs(uncorrected.gyro_rad_s[0]) > 0.02f);

    const float bias[3] = {uncorrected.gyro_rad_s[0], 0.0f, 0.0f};
    imu.set_gyro_bias(bias);

    Sample corrected;
    CHECK(imu.read(corrected, 0) == ImuStatus::kOk);
    CHECK_NEAR(corrected.gyro_rad_s[0], 0.0f, 1e-5);
}

TEST(accelerometer_angles_are_right_for_known_orientations) {
    float roll = 0.0f, pitch = 0.0f;

    const float level[3] = {0.0f, 0.0f, kGravity};
    accelerometer_angles(level, roll, pitch);
    CHECK_NEAR(roll, 0.0f, 1e-5);
    CHECK_NEAR(pitch, 0.0f, 1e-5);

    const float rolled[3] = {0.0f, kGravity, 0.0f};
    accelerometer_angles(rolled, roll, pitch);
    CHECK_NEAR(roll, kPi / 2.0f, 1e-5);

    const float pitched[3] = {-kGravity, 0.0f, 0.0f};
    accelerometer_angles(pitched, roll, pitch);
    CHECK_NEAR(pitch, kPi / 2.0f, 1e-5);
}

TEST(the_complementary_filter_settles_on_the_accelerometer) {
    ComplementaryFilter filter(0.5f);
    Sample sample;
    // Tilted thirty degrees in roll, perfectly still.
    const float angle = 30.0f * kPi / 180.0f;
    sample.accel_mps2[0] = 0.0f;
    sample.accel_mps2[1] = kGravity * std::sin(angle);
    sample.accel_mps2[2] = kGravity * std::cos(angle);

    Orientation state;
    for (int n = 0; n < 500; ++n) state = filter.update(sample, 0.01f);

    CHECK(state.trusted);
    CHECK_NEAR(state.roll_rad, angle, 0.01);
    CHECK(filter.stationary());
}

TEST(the_filter_does_not_trust_itself_until_it_has_settled) {
    ComplementaryFilter filter(0.5f);
    Sample sample;
    sample.accel_mps2[2] = kGravity;
    const Orientation first = filter.update(sample, 0.01f);
    CHECK(!first.trusted);

    Orientation state;
    for (int n = 0; n < 200; ++n) state = filter.update(sample, 0.01f);
    CHECK(state.trusted);
}

TEST(the_gyro_carries_the_estimate_through_a_shove) {
    // During a fast movement the accelerometer measures the arm's own
    // acceleration, not gravity. Blending it in then would drag the estimate
    // off, so it is ignored and the gyro carries the angle.
    ComplementaryFilter filter(0.5f);
    Sample level;
    level.accel_mps2[2] = kGravity;
    for (int n = 0; n < 500; ++n) filter.update(level, 0.01f);
    const float before = filter.state().roll_rad;

    Sample shoved = level;
    shoved.accel_mps2[1] = 6.0f * kGravity;  // well outside anything gravity can do
    Orientation state;
    for (int n = 0; n < 20; ++n) state = filter.update(shoved, 0.01f);

    CHECK_NEAR(state.roll_rad, before, 1e-4);
    CHECK(!filter.stationary());
}

TEST(the_filter_rejects_a_rotating_arm_as_not_stationary) {
    ComplementaryFilter filter(0.5f);
    Sample turning;
    turning.accel_mps2[2] = kGravity;
    turning.gyro_rad_s[2] = 1.0f;
    for (int n = 0; n < 100; ++n) filter.update(turning, 0.01f);
    CHECK(!filter.stationary());
}

TEST(coasting_through_a_dropout_stops_being_trusted) {
    ComplementaryFilter filter(0.5f);
    Sample sample;
    sample.accel_mps2[2] = kGravity;
    for (int n = 0; n < 500; ++n) filter.update(sample, 0.01f);
    CHECK(filter.state().trusted);

    Orientation state;
    for (int n = 0; n < 20; ++n) state = filter.coast(0.01f);
    CHECK(state.trusted);  // a fifth of a second is nothing

    for (int n = 0; n < 60; ++n) state = filter.coast(0.01f);
    CHECK(!state.trusted);  // past the time constant, the gyro has drifted
}

TEST(a_nonsense_timestep_leaves_the_filter_alone) {
    ComplementaryFilter filter(0.5f);
    Sample sample;
    sample.accel_mps2[2] = kGravity;
    for (int n = 0; n < 500; ++n) filter.update(sample, 0.01f);
    const Orientation before = filter.state();

    filter.update(sample, 0.0f);
    filter.update(sample, -1.0f);
    filter.coast(0.0f);
    const Orientation after = filter.state();

    CHECK_NEAR(after.roll_rad, before.roll_rad, 1e-9);
    CHECK_NEAR(after.pitch_rad, before.pitch_rad, 1e-9);
}

TEST_MAIN()
