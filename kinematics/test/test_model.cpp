#include "arm_kinematics/dynamics.hpp"
#include "arm_kinematics/model.hpp"
#include "test_framework.hpp"

using namespace ak;

TEST(the_model_has_six_named_joints) {
    const ArmModel model = a1_arm();
    CHECK_EQ(static_cast<int>(kNumJoints), 6);
    for (std::size_t i = 0; i < kNumJoints; ++i) CHECK(!model.joint_names[i].empty());

    // Names have to be unique: they become the URDF joint names and the
    // sensor_msgs/JointState fields and a duplicate would silently alias two
    // joints together.
    for (std::size_t i = 0; i < kNumJoints; ++i)
        for (std::size_t j = i + 1; j < kNumJoints; ++j)
            CHECK(model.joint_names[i] != model.joint_names[j]);
}

TEST(the_wrist_axes_are_concurrent) {
    CHECK(a1_arm().has_spherical_wrist());
}

TEST(every_joint_limit_is_well_formed) {
    const ArmModel model = a1_arm();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        CHECK(model.limits[i].lower < model.limits[i].upper);
        CHECK(model.limits[i].velocity > 0.0);
        CHECK(model.limits[i].effort > 0.0);
        // The home configuration has to be inside the stops or nothing can
        // start up.
        CHECK(model.limits[i].lower <= 0.0);
        CHECK(model.limits[i].upper >= 0.0);
    }
}

TEST(every_link_has_a_physically_possible_inertia) {
    const ArmModel model = a1_arm();
    auto check_body = [](const LinkInertia& link) {
        CHECK(link.mass > 0.0);
        const double ixx = link.inertia[0], iyy = link.inertia[1], izz = link.inertia[2];
        CHECK(ixx > 0.0);
        CHECK(iyy > 0.0);
        CHECK(izz > 0.0);
        // Triangle inequality on the principal moments. A tensor that breaks it
        // does not correspond to any real distribution of mass and the
        // dynamics will happily run on it and produce nonsense.
        CHECK(ixx + iyy >= izz);
        CHECK(iyy + izz >= ixx);
        CHECK(izz + ixx >= iyy);
    };
    for (const auto& link : model.links) check_body(link);
    check_body(model.tool);
}

TEST(the_whole_arm_weighs_what_a_wearable_arm_can_weigh) {
    // Not a physics check: a design constraint, written down where it will be
    // noticed if somebody adds a heavier actuator. A transhumeral prosthesis
    // above roughly two and a half kilograms is not tolerated for long and
    // docs/CLINICAL_LIMITATIONS.md says this one has never been worn.
    const ArmModel model = a1_arm();
    double total = model.tool.mass;
    for (const auto& link : model.links) total += link.mass;
    CHECK(total > 1.0);
    CHECK(total < 2.5);
}

TEST(clamping_reports_whether_it_changed_anything) {
    const ArmModel model = a1_arm();
    JointVector inside{};
    inside[0] = 0.5;
    CHECK(clamp_to_limits(model, inside));
    CHECK_NEAR(inside[0], 0.5, 1e-15);

    JointVector outside{};
    outside[0] = 99.0;
    CHECK(!clamp_to_limits(model, outside));
    CHECK_NEAR(outside[0], model.limits[0].upper, 1e-15);
    CHECK(within_limits(model, outside));
}

TEST(within_limits_respects_its_tolerance) {
    const ArmModel model = a1_arm();
    JointVector q{};
    q[1] = model.limits[1].upper + 1e-10;
    CHECK(within_limits(model, q, 1e-9));
    CHECK(!within_limits(model, q, 1e-12));
}

TEST(the_dh_table_only_uses_right_angle_twists) {
    // Not a requirement of the maths but of the mechanism: every axis on this
    // arm is either parallel or perpendicular to the one before it, which is
    // what lets the links be machined from stock rather than cast.
    const ArmModel model = a1_arm();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double alpha = model.dh[i].alpha;
        const double quarter_turns = alpha / 1.5707963267948966;
        CHECK_NEAR(quarter_turns, std::round(quarter_turns), 1e-15);
        CHECK(std::fabs(quarter_turns) <= 1.0);
    }
}

TEST_MAIN()
