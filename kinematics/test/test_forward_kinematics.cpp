#include <random>

#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/model.hpp"
#include "test_framework.hpp"

using namespace ak;

namespace {

constexpr double kPi = 3.14159265358979323846;

JointVector random_configuration(const ArmModel& model, std::mt19937_64& rng) {
    JointVector q{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        std::uniform_real_distribution<double> dist(model.limits[i].lower, model.limits[i].upper);
        q[i] = dist(rng);
    }
    return q;
}

}  // namespace

TEST(dh_transform_at_zero_is_the_link_offset) {
    const DhRow row{0.0, 0.25, 0.30, 0.0};
    const Transform t = dh_transform(row, 0.0);
    CHECK_NEAR(t.translation.x, 0.30, 1e-15);
    CHECK_NEAR(t.translation.y, 0.0, 1e-15);
    CHECK_NEAR(t.translation.z, 0.25, 1e-15);
    CHECK_NEAR(t.rotation(0, 0), 1.0, 1e-15);
    CHECK_NEAR(t.rotation(1, 1), 1.0, 1e-15);
    CHECK_NEAR(t.rotation(2, 2), 1.0, 1e-15);
}

TEST(dh_transform_rotations_stay_orthonormal) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(31);
    std::uniform_real_distribution<double> dist(-4.0, 4.0);
    for (int trial = 0; trial < 500; ++trial) {
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            const Transform t = dh_transform(model.dh[i], dist(rng));
            const Mat3 should_be_identity = t.rotation * t.rotation.transposed();
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    CHECK_NEAR(should_be_identity(r, c), r == c ? 1.0 : 0.0, 1e-13);
        }
    }
}

TEST(the_home_pose_is_the_arm_out_and_the_forearm_down) {
    // At q = 0 the shoulder lifts the chain to d1, the upper arm reaches a2
    // along the base x axis and the forearm and wrist then hang straight down
    // by d4 + d6. This is the one pose that can be read off the DH table by
    // hand, which makes it the anchor everything else is checked against.
    const ArmModel model = a1_arm();
    const JointVector zero{};
    const Transform pose = forward_kinematics(model, zero);

    const double d1 = model.dh[0].d;
    const double a2 = model.dh[1].a;
    const double d4 = model.dh[3].d;
    const double d6 = model.dh[5].d;

    CHECK_NEAR(pose.translation.x, a2, 1e-12);
    CHECK_NEAR(pose.translation.y, 0.0, 1e-12);
    CHECK_NEAR(pose.translation.z, d1 - d4 - d6, 1e-12);

    // The approach axis points straight down, so the terminal device faces the
    // bench in the home pose.
    const Vec3 approach = pose.axis(2);
    CHECK_NEAR(approach.x, 0.0, 1e-12);
    CHECK_NEAR(approach.y, 0.0, 1e-12);
    CHECK_NEAR(approach.z, -1.0, 1e-12);

    // And the wrist centre sits one d6 above it.
    const Vec3 centre = wrist_centre(model, zero);
    CHECK_NEAR(centre.x, a2, 1e-12);
    CHECK_NEAR(centre.z, d1 - d4, 1e-12);
}

TEST(the_arm_never_reaches_further_than_the_chain_is_long) {
    const ArmModel model = a1_arm();
    const double longest = model.dh[0].d + model.dh[1].a + model.dh[3].d + model.dh[5].d;

    std::mt19937_64 rng(37);
    for (int trial = 0; trial < 20000; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const Transform pose = forward_kinematics(model, q);
        CHECK(norm(pose.translation) <= longest + 1e-9);
    }
}

TEST(rotating_the_base_joint_rotates_the_whole_arm) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(41);
    for (int trial = 0; trial < 200; ++trial) {
        JointVector q = random_configuration(model, rng);
        const Transform before = forward_kinematics(model, q);

        const double delta = 0.3;
        q[0] = wrap_angle(q[0] + delta);
        const Transform after = forward_kinematics(model, q);

        // Joint 1 turns about the base z, so the tool position must simply
        // rotate about that axis by the same angle.
        const Transform expected_rotation{rotation_z(delta), Vec3{}};
        const Vec3 predicted = expected_rotation.apply(before.translation);
        CHECK_NEAR(norm(after.translation - predicted), 0.0, 1e-12);
    }
}

TEST(the_wrist_centre_does_not_move_when_the_wrist_turns) {
    // The whole analytic inverse solution rests on this. If joints 4, 5 and 6
    // moved the wrist centre, the position and orientation sub-problems would
    // not separate.
    const ArmModel model = a1_arm();
    CHECK(model.has_spherical_wrist());

    std::mt19937_64 rng(43);
    for (int trial = 0; trial < 2000; ++trial) {
        JointVector q = random_configuration(model, rng);
        const Vec3 before = wrist_centre(model, q);

        const JointVector wrist_only = random_configuration(model, rng);
        q[3] = wrist_only[3];
        q[4] = wrist_only[4];
        q[5] = wrist_only[5];
        const Vec3 after = wrist_centre(model, q);

        CHECK_NEAR(norm(after - before), 0.0, 1e-12);
    }
}

TEST(the_jacobian_matches_numerical_differentiation) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(47);
    const double h = 1e-6;

    for (int trial = 0; trial < 300; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const auto j = jacobian(model, q);
        const Transform base = forward_kinematics(model, q);

        for (std::size_t i = 0; i < kNumJoints; ++i) {
            JointVector forward = q, backward = q;
            forward[i] += h;
            backward[i] -= h;
            const Transform tp = forward_kinematics(model, forward);
            const Transform tm = forward_kinematics(model, backward);

            const Vec3 linear = (tp.translation - tm.translation) * (1.0 / (2.0 * h));
            CHECK_NEAR(j(0, i), linear.x, 1e-6);
            CHECK_NEAR(j(1, i), linear.y, 1e-6);
            CHECK_NEAR(j(2, i), linear.z, 1e-6);

            // Angular columns: the rotation vector taking R(q-h) to R(q+h),
            // over the interval.
            const Vec3 angular = rotation_error(tm.rotation, tp.rotation) * (1.0 / (2.0 * h));
            CHECK_NEAR(j(3, i), angular.x, 1e-6);
            CHECK_NEAR(j(4, i), angular.y, 1e-6);
            CHECK_NEAR(j(5, i), angular.z, 1e-6);
        }
        CHECK(std::isfinite(base.translation.x));
    }
}

TEST(manipulability_collapses_at_the_elbow_singularity) {
    const ArmModel model = a1_arm();

    // Elbow straight: joints 2 and 3 line the forearm up with the upper arm and
    // the arm loses the ability to move along its own length.
    //
    // Joint 5 is held away from zero throughout. At q5 = 0 joints 4 and 6 are
    // collinear and the wrist is in its own singularity, which would mask the
    // elbow one this test is about.
    JointVector stretched{};
    stretched[1] = 0.0;
    stretched[2] = kPi / 2.0;
    stretched[4] = 0.8;
    const double at_singularity = manipulability(jacobian(model, stretched));

    JointVector folded = stretched;
    folded[2] = kPi / 2.0 - 0.7;
    const double away = manipulability(jacobian(model, folded));

    CHECK(at_singularity < 1e-6);
    CHECK(away > 1e-2);
    CHECK(away > at_singularity * 100.0);
}

TEST(link_transforms_compose_to_the_tool_pose) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(53);
    for (int trial = 0; trial < 500; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const auto frames = link_transforms(model, q);

        Transform running = Transform::identity();
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            running = running * dh_transform(model.dh[i], q[i]);
            CHECK_NEAR(norm(running.translation - frames[i].translation), 0.0, 1e-14);
        }
        const Transform pose = forward_kinematics(model, q);
        CHECK_NEAR(norm(pose.translation - running.translation), 0.0, 1e-14);
    }
}

TEST_MAIN()
