#include <random>

#include "arm_kinematics/trajectory.hpp"
#include "test_framework.hpp"

using namespace ak;

namespace {

JointVector random_configuration(const ArmModel& model, std::mt19937_64& rng, double margin) {
    JointVector q{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        std::uniform_real_distribution<double> dist(model.limits[i].lower * margin,
                                                    model.limits[i].upper * margin);
        q[i] = dist(rng);
    }
    return q;
}

const ProfileType kProfiles[] = {ProfileType::kQuintic, ProfileType::kTrapezoidal};

}  // namespace

TEST(a_plan_starts_and_ends_where_it_was_told_to) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(301);
    for (ProfileType profile : kProfiles) {
        for (int trial = 0; trial < 200; ++trial) {
            const JointVector start = random_configuration(model, rng, 0.8);
            const JointVector goal = random_configuration(model, rng, 0.8);
            const JointTrajectory trajectory = plan_joint_move(model, start, goal, profile);

            const TrajectoryPoint first = trajectory.sample(0.0);
            const TrajectoryPoint last = trajectory.sample(trajectory.duration());
            for (std::size_t i = 0; i < kNumJoints; ++i) {
                CHECK_NEAR(first.q[i], start[i], 1e-12);
                CHECK_NEAR(last.q[i], goal[i], 1e-12);
            }
        }
    }
}

TEST(a_plan_starts_and_ends_at_rest) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(302);
    for (ProfileType profile : kProfiles) {
        for (int trial = 0; trial < 200; ++trial) {
            const JointVector start = random_configuration(model, rng, 0.8);
            const JointVector goal = random_configuration(model, rng, 0.8);
            const JointTrajectory trajectory = plan_joint_move(model, start, goal, profile);

            // Sampled just inside the ends rather than at them, because at the
            // ends the sample function short-circuits to the endpoint.
            const double eps = trajectory.duration() * 1e-6;
            const TrajectoryPoint first = trajectory.sample(eps);
            const TrajectoryPoint last = trajectory.sample(trajectory.duration() - eps);
            for (std::size_t i = 0; i < kNumJoints; ++i) {
                CHECK_NEAR(first.qd[i], 0.0, 1e-3);
                CHECK_NEAR(last.qd[i], 0.0, 1e-3);
            }
        }
    }
}

TEST(a_plan_never_exceeds_the_scaled_joint_limits) {
    const ArmModel model = a1_arm();
    PlannerLimits limits;
    std::mt19937_64 rng(303);

    for (ProfileType profile : kProfiles) {
        for (int trial = 0; trial < 300; ++trial) {
            const JointVector start = random_configuration(model, rng, 0.9);
            const JointVector goal = random_configuration(model, rng, 0.9);
            const JointTrajectory trajectory = plan_joint_move(model, start, goal, profile, limits);

            for (const auto& point : trajectory.discretise(400)) {
                for (std::size_t i = 0; i < kNumJoints; ++i) {
                    const double velocity_ceiling = model.limits[i].velocity * limits.velocity_scale;
                    const double accel_ceiling =
                        limits.max_acceleration * limits.acceleration_scale;
                    // A whisker of slack for the discretisation, not for the
                    // profile: the peaks are exact and known in closed form.
                    CHECK(std::fabs(point.qd[i]) <= velocity_ceiling * 1.001);
                    CHECK(std::fabs(point.qdd[i]) <= accel_ceiling * 1.001);
                }
            }
        }
    }
}

TEST(the_quintic_profile_has_no_acceleration_step_at_the_ends) {
    // What separates it from the trapezoid and the reason it is the default.
    const ArmModel model = a1_arm();
    JointVector start{};
    JointVector goal{};
    goal[1] = 1.0;

    const JointTrajectory quintic = plan_joint_move(model, start, goal, ProfileType::kQuintic);
    const JointTrajectory trapezoid =
        plan_joint_move(model, start, goal, ProfileType::kTrapezoidal);

    const double eps = 1e-6;
    CHECK_NEAR(quintic.sample(eps).qdd[1], 0.0, 1e-3);
    CHECK_NEAR(quintic.sample(quintic.duration() - eps).qdd[1], 0.0, 1e-3);
    CHECK(std::fabs(trapezoid.sample(eps).qdd[1]) > 0.5);
}

TEST(the_trapezoid_gets_there_sooner_than_the_quintic) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(304);
    for (int trial = 0; trial < 100; ++trial) {
        const JointVector start = random_configuration(model, rng, 0.8);
        const JointVector goal = random_configuration(model, rng, 0.8);
        const double quintic =
            plan_joint_move(model, start, goal, ProfileType::kQuintic).duration();
        const double trapezoid =
            plan_joint_move(model, start, goal, ProfileType::kTrapezoidal).duration();
        CHECK(trapezoid <= quintic + 1e-12);
    }
}

TEST(every_joint_arrives_at_the_same_moment) {
    // The point of synchronising. A joint that finishes early and waits traces
    // a path through joint space nobody planned.
    const ArmModel model = a1_arm();
    JointVector start{};
    JointVector goal{};
    goal[0] = 1.4;   // a long way for the slow shoulder
    goal[5] = 0.05;  // almost nothing for the fast wrist

    const JointTrajectory trajectory = plan_joint_move(model, start, goal);
    const double half = trajectory.duration() * 0.5;
    const TrajectoryPoint mid = trajectory.sample(half);

    // At the halfway point in time, every joint is halfway through its own
    // displacement, because they share one normalised profile.
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double fraction = goal[i] == 0.0 ? 0.5 : (mid.q[i] - start[i]) / (goal[i] - start[i]);
        CHECK_NEAR(fraction, 0.5, 1e-9);
    }
}

TEST(sampling_outside_the_move_holds_the_endpoints) {
    const ArmModel model = a1_arm();
    JointVector start{};
    JointVector goal{};
    goal[2] = -0.8;
    const JointTrajectory trajectory = plan_joint_move(model, start, goal);

    const TrajectoryPoint before = trajectory.sample(-1.0);
    const TrajectoryPoint after = trajectory.sample(trajectory.duration() + 5.0);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        CHECK_NEAR(before.q[i], start[i], 1e-15);
        CHECK_NEAR(after.q[i], goal[i], 1e-15);
        CHECK_NEAR(after.qd[i], 0.0, 1e-15);
    }
}

TEST(a_zero_length_move_still_has_a_duration) {
    const ArmModel model = a1_arm();
    const JointVector q{};
    const JointTrajectory trajectory = plan_joint_move(model, q, q);
    CHECK(trajectory.duration() > 0.0);
    CHECK(!trajectory.empty());
}

TEST(discretise_returns_one_more_point_than_intervals) {
    const ArmModel model = a1_arm();
    JointVector goal{};
    goal[1] = 0.6;
    const JointTrajectory trajectory = plan_joint_move(model, JointVector{}, goal);
    const auto points = trajectory.discretise(100);
    CHECK_EQ(static_cast<int>(points.size()), 101);
    CHECK_NEAR(points.front().time, 0.0, 1e-15);
    CHECK_NEAR(points.back().time, trajectory.duration(), 1e-12);
    for (std::size_t i = 1; i < points.size(); ++i) CHECK(points[i].time > points[i - 1].time);
}

TEST(pose_interpolation_hits_both_ends_and_stays_rigid) {
    const ArmModel model = a1_arm();
    JointVector a{}, b{};
    a[1] = 0.3; a[2] = -0.6; a[4] = 0.5;
    b[1] = 0.9; b[2] = -1.2; b[4] = -0.4; b[0] = 0.7;

    const Transform from = forward_kinematics(model, a);
    const Transform to = forward_kinematics(model, b);

    CHECK_NEAR(norm(interpolate_pose(from, to, 0.0).translation - from.translation), 0.0, 1e-14);
    CHECK_NEAR(norm(interpolate_pose(from, to, 1.0).translation - to.translation), 0.0, 1e-14);
    CHECK_NEAR(norm(rotation_error(interpolate_pose(from, to, 1.0).rotation, to.rotation)), 0.0,
               1e-12);

    for (int step = 0; step <= 20; ++step) {
        const Transform mid = interpolate_pose(from, to, step / 20.0);
        const Mat3 check = mid.rotation * mid.rotation.transposed();
        for (std::size_t r = 0; r < 3; ++r)
            for (std::size_t c = 0; c < 3; ++c)
                CHECK_NEAR(check(r, c), r == c ? 1.0 : 0.0, 1e-12);
    }
}

TEST(a_cartesian_move_traces_a_straight_line) {
    const ArmModel model = a1_arm();
    JointVector start{};
    start[1] = 0.5;
    start[2] = -0.9;
    start[4] = 0.6;

    const Transform from = forward_kinematics(model, start);
    Transform to = from;
    to.translation = to.translation + Vec3{0.08, 0.05, 0.06};

    const CartesianPlan plan = plan_cartesian_line(model, start, to, 1.2, 60);
    CHECK(plan.status == IkStatus::kSuccess);
    CHECK_EQ(static_cast<int>(plan.points.size()), 60);

    const Vec3 direction = to.translation - from.translation;
    const double length = norm(direction);
    for (const auto& point : plan.points) {
        const Vec3 achieved = forward_kinematics(model, point.q).translation;
        const Vec3 offset = achieved - from.translation;
        const double along = dot(offset, direction) / length;
        const Vec3 perpendicular = offset - direction * (along / length);
        // Off-axis deviation, which is what "straight" means here. Every
        // waypoint is solved exactly, so this is bounded by solver precision.
        CHECK(norm(perpendicular) < 1e-9);
        CHECK(along > -1e-9);
        CHECK(along < length + 1e-9);
    }

    CHECK_NEAR(norm(forward_kinematics(model, plan.points.back().q).translation - to.translation),
               0.0, 1e-10);
}

TEST(a_cartesian_move_reports_where_it_left_the_workspace) {
    const ArmModel model = a1_arm();
    JointVector start{};
    start[1] = 0.4;
    start[2] = -0.8;

    Transform to = forward_kinematics(model, start);
    to.translation = to.translation + Vec3{3.0, 0.0, 0.0};

    const CartesianPlan plan = plan_cartesian_line(model, start, to, 1.0, 40);
    CHECK(plan.status != IkStatus::kSuccess);
    CHECK(plan.failed_at > 0);
    CHECK(plan.failed_at < 40);
}

TEST(a_cartesian_move_stays_on_one_branch) {
    // Seeding each waypoint from the previous solution is what stops the elbow
    // flipping halfway along a path. Without it the joint rates would spike.
    const ArmModel model = a1_arm();
    JointVector start{};
    start[0] = 0.2;
    start[1] = 0.6;
    start[2] = -1.0;
    start[4] = 0.7;

    Transform to = forward_kinematics(model, start);
    to.translation = to.translation + Vec3{0.10, -0.06, 0.04};

    const CartesianPlan plan = plan_cartesian_line(model, start, to, 1.5, 80);
    CHECK(plan.status == IkStatus::kSuccess);

    for (std::size_t i = 1; i < plan.points.size(); ++i) {
        for (std::size_t j = 0; j < kNumJoints; ++j) {
            const double step = std::fabs(wrap_angle(plan.points[i].q[j] - plan.points[i - 1].q[j]));
            CHECK(step < 0.15);
        }
    }
    CHECK(plan.max_joint_velocity > 0.0);
    CHECK(plan.max_joint_velocity < 4.0);
}

TEST(a_cartesian_move_is_rejected_rather_than_fudged_when_it_is_malformed) {
    const ArmModel model = a1_arm();
    const Transform pose = forward_kinematics(model, JointVector{});
    CHECK(plan_cartesian_line(model, JointVector{}, pose, 1.0, 1).status != IkStatus::kSuccess);
    CHECK(plan_cartesian_line(model, JointVector{}, pose, 0.0, 20).status != IkStatus::kSuccess);
}

TEST_MAIN()
