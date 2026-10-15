#include <cmath>

#include "arm_control/joint_controller.hpp"
#include "test_framework.hpp"

using namespace arm_control;

namespace {

ak::JointVector pose(double a, double b, double c, double d, double e, double f) {
    return ak::JointVector{{a, b, c, d, e, f}};
}

}  // namespace

TEST(loop_timing_reports_the_period_it_was_driven_at) {
    LoopTiming timing;
    const double nominal = 0.01;
    for (int i = 0; i < 1000; ++i) timing.record(i * nominal, nominal, 0.25);

    CHECK_EQ(static_cast<int>(timing.iterations()), 1000);
    CHECK_NEAR(timing.mean_period_ms(), 10.0, 1e-6);
    CHECK_NEAR(timing.max_period_ms(), 10.0, 1e-6);
    CHECK_NEAR(timing.jitter_rms_ms(), 0.0, 1e-6);
    CHECK_EQ(static_cast<int>(timing.overruns()), 0);
}

TEST(loop_timing_notices_jitter_and_counts_overruns) {
    LoopTiming timing;
    const double nominal = 0.01;
    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        timing.record(t, nominal, 0.25);
        // Every tenth tick arrives half a period late.
        t += (i % 10 == 0) ? nominal * 1.5 : nominal;
    }
    CHECK(timing.jitter_rms_ms() > 0.5);
    CHECK(timing.max_period_ms() > 14.0);
    CHECK(timing.overruns() >= 9);
}

TEST(loop_timing_resets_cleanly) {
    LoopTiming timing;
    for (int i = 0; i < 10; ++i) timing.record(i * 0.01, 0.01, 0.25);
    timing.reset();
    CHECK_EQ(static_cast<int>(timing.iterations()), 0);
    CHECK_NEAR(timing.mean_period_ms(), 0.0, 1e-12);
    CHECK_NEAR(timing.jitter_rms_ms(), 0.0, 1e-12);
}

TEST(the_controller_holds_its_reference_with_nothing_to_follow) {
    const ak::ArmModel model = ak::a1_arm();
    JointController controller(model, ControllerConfig{});

    const ak::JointVector measured{};
    const ControlCommand first = controller.update(0.0, measured, measured);
    const ControlCommand second = controller.update(0.01, measured, measured);
    CHECK(!first.trajectory_active);
    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        CHECK_NEAR(first.position[i], second.position[i], 1e-12);
}

TEST(the_controller_follows_a_trajectory_from_end_to_end) {
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;  // isolate the trajectory from the compensation
    JointController controller(model, config);

    const ak::JointVector start{};
    const ak::JointVector goal = pose(0.4, 0.7, -1.0, 0.5, 0.6, -0.3);
    const ak::JointTrajectory trajectory = ak::plan_joint_move(model, start, goal);
    controller.set_trajectory(trajectory, 10.0);

    ControlCommand command;
    bool saw_finish = false;
    for (double t = 10.0; t <= 10.0 + trajectory.duration() + 0.05; t += 0.01) {
        command = controller.update(t, start, ak::JointVector{});
        saw_finish = saw_finish || command.trajectory_finished;
    }

    CHECK(saw_finish);
    CHECK(!controller.trajectory_active());
    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        CHECK_NEAR(command.reference[i], goal[i], 1e-9);
}

TEST(the_reference_starts_where_the_trajectory_starts) {
    // A controller that jumped to the first sampled point rather than the
    // trajectory's own start would produce a step on every new gesture.
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;
    JointController controller(model, config);

    const ak::JointVector start = pose(0.1, 0.5, -0.9, 0.0, 0.2, 0.0);
    const ak::JointVector goal = pose(-0.3, 0.9, -1.3, 0.4, -0.1, 0.2);
    controller.set_trajectory(ak::plan_joint_move(model, start, goal), 5.0);

    const ControlCommand command = controller.update(5.0, start, ak::JointVector{});
    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        CHECK_NEAR(command.reference[i], start[i], 1e-12);
}

TEST(a_new_trajectory_supersedes_the_one_running) {
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;
    JointController controller(model, config);

    const ak::JointVector start{};
    const ak::JointVector first_goal = pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const ak::JointVector second_goal = pose(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    controller.set_trajectory(ak::plan_joint_move(model, start, first_goal), 0.0);
    for (double t = 0.0; t < 0.2; t += 0.01) controller.update(t, start, ak::JointVector{});
    const ak::JointVector midway = controller.reference();

    controller.set_trajectory(ak::plan_joint_move(model, midway, second_goal), 0.2);
    ControlCommand command;
    for (double t = 0.2; t < 3.0; t += 0.01) command = controller.update(t, start, ak::JointVector{});

    CHECK_NEAR(command.reference[0], second_goal[0], 1e-6);
}

TEST(gravity_compensation_leans_the_command_into_the_load) {
    // The whole point: a position servo with a load on it sits short of where
    // it was told and the compensation is what makes up the difference. Its
    // sign has to follow the load or it would double the error instead.
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.012;
    JointController controller(model, config);

    // Arm horizontal and straight out: the worst case for the shoulder.
    const ak::JointVector stretched = pose(0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0);
    controller.set_trajectory(ak::JointTrajectory(stretched, stretched, 0.5,
                                                  ak::ProfileType::kQuintic),
                              0.0);
    const ControlCommand command = controller.update(0.0, stretched, ak::JointVector{});

    const ak::JointVector expected = ak::gravity_torque(model, stretched);
    CHECK(std::fabs(expected[1]) > 3.0);
    // Same sign as the torque it is compensating.
    CHECK(command.compensation[1] * expected[1] > 0.0);
    CHECK_NEAR(command.compensation[1], expected[1] * config.compliance_rad_per_nm, 1e-9);
    CHECK_NEAR(command.position[1], command.reference[1] + command.compensation[1], 1e-9);
}

TEST(the_velocity_lead_cancels_a_servos_following_error) {
    // A position servo lags while it is moving, by an amount proportional to
    // speed. Nothing on this arm can measure that lag, so the controller
    // commands a position the servo will lag onto instead. The lead has to
    // point the way the arm is going and scale with how fast it is going.
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;  // isolate the lead from the gravity term
    config.velocity_lead_s = 0.064;
    JointController controller(model, config);

    const ak::JointVector start{};
    const ak::JointVector goal = pose(1.2, 0.0, 0.0, 0.0, 0.0, 0.0);
    const ak::JointTrajectory trajectory = plan_joint_move(model, start, goal);
    controller.set_trajectory(trajectory, 0.0);

    // Halfway through, where a quintic is at its fastest.
    const ControlCommand command = controller.update(trajectory.duration() * 0.5, start,
                                                     ak::JointVector{});
    CHECK(command.velocity[0] > 0.5);
    CHECK_NEAR(command.lead[0], command.velocity[0] * config.velocity_lead_s, 1e-12);
    CHECK_NEAR(command.position[0], command.reference[0] + command.lead[0], 1e-12);

    // Moving the other way, the lead goes the other way.
    JointController reverse(model, config);
    reverse.set_trajectory(plan_joint_move(model, goal, start), 0.0);
    const ControlCommand back = reverse.update(trajectory.duration() * 0.5, goal,
                                               ak::JointVector{});
    CHECK(back.lead[0] < 0.0);
}

TEST(the_velocity_lead_is_zero_at_rest_and_at_both_ends_of_a_move) {
    // It must not leave a standing offset. A lead that did would move the arm
    // away from where it was told to stop.
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;
    JointController controller(model, config);

    const ak::JointVector start{};
    const ak::JointVector goal = pose(0.8, 0.4, -0.6, 0.0, 0.2, 0.0);
    const ak::JointTrajectory trajectory = plan_joint_move(model, start, goal);
    controller.set_trajectory(trajectory, 0.0);

    const ControlCommand first = controller.update(1e-6, start, ak::JointVector{});
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) CHECK_NEAR(first.lead[i], 0.0, 1e-3);

    ControlCommand last;
    for (double t = 0.0; t <= trajectory.duration() + 0.1; t += 0.01)
        last = controller.update(t, goal, ak::JointVector{});
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        CHECK_NEAR(last.lead[i], 0.0, 1e-9);
        CHECK_NEAR(last.position[i], goal[i], 1e-9);
    }
}

TEST(gravity_compensation_is_zero_when_it_is_turned_off) {
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;
    JointController controller(model, config);

    const ak::JointVector stretched = pose(0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0);
    controller.set_trajectory(ak::JointTrajectory(stretched, stretched, 0.5,
                                                  ak::ProfileType::kQuintic),
                              0.0);
    const ControlCommand command = controller.update(0.0, stretched, ak::JointVector{});
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        CHECK_NEAR(command.compensation[i], 0.0, 1e-12);
        CHECK_NEAR(command.position[i], command.reference[i] + command.lead[i], 1e-12);
    }
}

TEST(gravity_compensation_is_capped) {
    // A modelling error must not turn into a large commanded offset. An absurd
    // compliance stands in for one.
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 10.0;
    config.max_compensation_rad = 0.25;
    JointController controller(model, config);

    const ak::JointVector stretched = pose(0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0);
    controller.set_trajectory(ak::JointTrajectory(stretched, stretched, 0.5,
                                                  ak::ProfileType::kQuintic),
                              0.0);
    const ControlCommand command = controller.update(0.0, stretched, ak::JointVector{});
    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        CHECK(std::fabs(command.compensation[i]) <= config.max_compensation_rad + 1e-12);
}

TEST(the_command_is_always_inside_the_joint_limits) {
    // The firmware clamps too. Doing it here as well means the telemetry
    // reports what was asked for rather than what happened to survive.
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 10.0;
    config.max_compensation_rad = 5.0;
    JointController controller(model, config);

    ak::JointVector extreme;
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) extreme[i] = model.limits[i].upper;
    controller.set_trajectory(ak::JointTrajectory(extreme, extreme, 0.5, ak::ProfileType::kQuintic),
                              0.0);

    const ControlCommand command = controller.update(0.0, extreme, ak::JointVector{});
    CHECK(ak::within_limits(model, command.position, 1e-9));
}

TEST(tracking_error_is_recorded_against_the_reference) {
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;
    JointController controller(model, config);

    const ak::JointVector reference = pose(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
    controller.set_trajectory(ak::JointTrajectory(reference, reference, 1.0,
                                                  ak::ProfileType::kQuintic),
                              0.0);

    // The arm is somewhere else entirely.
    const ak::JointVector measured{};
    controller.update(0.0, measured, ak::JointVector{});
    CHECK_NEAR(controller.peak_tracking_error()[0], 0.5, 1e-9);
}

TEST(holding_stops_the_trajectory_without_moving_the_reference) {
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.compliance_rad_per_nm = 0.0;
    JointController controller(model, config);

    const ak::JointVector start{};
    const ak::JointVector goal = pose(1.0, 0.5, -0.5, 0.0, 0.0, 0.0);
    controller.set_trajectory(ak::plan_joint_move(model, start, goal), 0.0);
    for (double t = 0.0; t < 0.3; t += 0.01) controller.update(t, start, ak::JointVector{});

    const ak::JointVector before = controller.reference();
    controller.hold();
    const ControlCommand command = controller.update(0.31, start, ak::JointVector{});

    CHECK(!controller.trajectory_active());
    for (std::size_t i = 0; i < ak::kNumJoints; ++i)
        CHECK_NEAR(command.reference[i], before[i], 1e-12);
}

TEST(the_controller_measures_its_own_loop) {
    const ak::ArmModel model = ak::a1_arm();
    ControllerConfig config;
    config.rate_hz = 100.0;
    JointController controller(model, config);

    const ak::JointVector measured{};
    for (int i = 0; i < 500; ++i) controller.update(i * 0.01, measured, measured);

    const LoopTiming& timing = controller.timing();
    CHECK_EQ(static_cast<int>(timing.iterations()), 500);
    CHECK_NEAR(timing.mean_period_ms(), 10.0, 1e-6);
    CHECK_EQ(static_cast<int>(timing.overruns()), 0);
}

TEST_MAIN()
