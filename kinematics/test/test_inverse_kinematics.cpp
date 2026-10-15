#include <random>

#include "arm_kinematics/inverse_kinematics.hpp"
#include "test_framework.hpp"

using namespace ak;

namespace {

constexpr double kPi = 3.14159265358979323846;

JointVector random_configuration(const ArmModel& model, std::mt19937_64& rng, double margin) {
    JointVector q{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        std::uniform_real_distribution<double> dist(model.limits[i].lower * margin,
                                                    model.limits[i].upper * margin);
        q[i] = dist(rng);
    }
    return q;
}

double pose_position_error(const ArmModel& model, const JointVector& q, const Transform& target) {
    return norm(forward_kinematics(model, q).translation - target.translation);
}

double pose_orientation_error(const ArmModel& model, const JointVector& q,
                              const Transform& target) {
    return norm(rotation_error(forward_kinematics(model, q).rotation, target.rotation));
}

}  // namespace

TEST(analytic_solver_reproduces_the_pose_it_was_given) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(101);
    const JointVector neutral{};

    int solved = 0;
    for (int trial = 0; trial < 5000; ++trial) {
        const JointVector q = random_configuration(model, rng, 1.0);
        const Transform target = forward_kinematics(model, q);
        const IkResult result = inverse_kinematics(model, target, neutral);
        CHECK(result.ok());
        if (!result.ok()) continue;
        ++solved;
        CHECK_NEAR(pose_position_error(model, result.q, target), 0.0, 1e-12);
        CHECK_NEAR(pose_orientation_error(model, result.q, target), 0.0, 1e-9);
        CHECK(within_limits(model, result.q, 1e-9));
    }
    CHECK_EQ(solved, 5000);
}

TEST(every_returned_branch_reaches_the_same_pose) {
    // Eight branches, one pose. If any of them is wrong the arm will silently
    // pick it whenever it happens to be nearest the seed.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(103);
    const JointVector neutral{};

    for (int trial = 0; trial < 1000; ++trial) {
        const JointVector q = random_configuration(model, rng, 0.9);
        const Transform target = forward_kinematics(model, q);
        const auto branches = inverse_kinematics_all(model, target, neutral);
        CHECK(!branches.empty());
        for (const auto& branch : branches) {
            CHECK_NEAR(pose_position_error(model, branch.q, target), 0.0, 1e-11);
            CHECK_NEAR(pose_orientation_error(model, branch.q, target), 0.0, 1e-8);
        }
    }
}

TEST(the_solver_finds_eight_distinct_branches_away_from_singularities) {
    const ArmModel model = a1_arm();
    // Well clear of the shoulder axis, the elbow stretch and the wrist flat.
    JointVector q{};
    q[0] = 0.4; q[1] = 0.5; q[2] = -0.9; q[3] = 0.7; q[4] = 0.9; q[5] = -0.6;

    const Transform target = forward_kinematics(model, q);
    const auto branches = inverse_kinematics_all(model, target, JointVector{});
    CHECK_EQ(static_cast<int>(branches.size()), 8);

    // No two branches are the same set of angles.
    int identical = 0;
    for (std::size_t i = 0; i < branches.size(); ++i) {
        for (std::size_t j = i + 1; j < branches.size(); ++j) {
            double difference = 0.0;
            for (std::size_t k = 0; k < kNumJoints; ++k)
                difference += std::fabs(wrap_angle(branches[i].q[k] - branches[j].q[k]));
            if (difference < 1e-6) ++identical;
        }
    }
    CHECK_EQ(identical, 0);
}

TEST(the_solver_picks_the_branch_nearest_the_seed) {
    // The contract is that no other in-limits branch is closer to the seed,
    // not that the answer equals the configuration the pose was generated
    // from. Those are different statements: near the elbow stretch the two
    // elbow branches converge on each other and near the shoulder axis the
    // forward and backward branches do, so the truth is regularly not the
    // nearest solution. Both are exact and picking the nearer one is the
    // right behaviour.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(107);

    int checked = 0;
    for (int trial = 0; trial < 2000; ++trial) {
        const JointVector q = random_configuration(model, rng, 0.85);
        const Transform target = forward_kinematics(model, q);

        JointVector seed = q;
        std::uniform_real_distribution<double> nudge(-0.05, 0.05);
        for (std::size_t i = 0; i < kNumJoints; ++i) seed[i] += nudge(rng);

        const IkResult result = inverse_kinematics(model, target, seed);
        CHECK(result.ok());
        if (!result.ok()) continue;
        ++checked;

        const double chosen = branch_seed_cost(result.q, seed);
        for (const auto& branch : inverse_kinematics_all(model, target, seed)) {
            if (!branch.within_limits) continue;
            CHECK(chosen <= branch_seed_cost(branch.q, seed) + 1e-12);
        }
        // And it is never worse than staying where the arm already is, which
        // is the property that stops a small pose change causing a big move.
        CHECK(chosen <= branch_seed_cost(q, seed) + 1e-12);
    }
    CHECK_EQ(checked, 2000);
}

TEST(a_small_pose_change_produces_a_small_joint_change) {
    // What "nearest the seed" is for: nudging the target by a millimetre must
    // not move a joint by a radian.
    //
    // Only away from singularities and the Jacobian is asked rather than
    // guessed at. Close to one the joint rates genuinely do blow up, because
    // dq = J^-1 dx and J is losing rank; a solver that hid that would be
    // lying about the mechanism. The bound below is the conditioning bound,
    // with a factor of three of slack for the fact that the step is finite.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(108);
    const Vec3 nudge{0.001, -0.0005, 0.0008};
    const double step = norm(nudge);

    int checked = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        const JointVector q = random_configuration(model, rng, 0.7);
        const double sigma = manipulability(jacobian(model, q));
        if (sigma < 0.05) continue;

        const Transform from = forward_kinematics(model, q);
        Transform to = from;
        to.translation = to.translation + nudge;

        const IkResult first = inverse_kinematics(model, from, q);
        const IkResult second = inverse_kinematics(model, to, first.q);
        if (!first.ok() || !second.ok()) continue;
        ++checked;

        for (std::size_t i = 0; i < kNumJoints; ++i)
            CHECK(std::fabs(wrap_angle(second.q[i] - first.q[i])) < 3.0 * step / sigma);
    }
    // The guard must not have thrown away the whole sample.
    CHECK(checked > 1000);
}

TEST(a_pose_beyond_reach_is_reported_not_approximated) {
    const ArmModel model = a1_arm();
    Transform far_away;
    far_away.translation = {2.0, 0.0, 0.0};
    const IkResult result = inverse_kinematics(model, far_away, JointVector{});
    CHECK(!result.ok());
    CHECK(result.status == IkStatus::kUnreachable);
}

TEST(a_pose_inside_the_shell_but_outside_the_stops_is_reported_as_such) {
    // Directly behind the shoulder and low: geometrically inside the reach
    // envelope but every branch needs a joint past its stop.
    const ArmModel model = a1_arm();
    ArmModel narrowed = model;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        narrowed.limits[i].lower = -0.05;
        narrowed.limits[i].upper = 0.05;
    }

    JointVector far_from_home{};
    far_from_home[1] = 1.2;
    far_from_home[2] = -1.4;
    const Transform target = forward_kinematics(model, far_from_home);

    const IkResult result = inverse_kinematics(narrowed, target, JointVector{});
    CHECK(!result.ok());
    CHECK(result.status == IkStatus::kOutsideLimits);
}

TEST(the_numerical_solver_agrees_with_the_analytic_one) {
    // Two implementations that share nothing but the forward kinematics. If
    // they agree to nine decimal places over thousands of poses, the closed
    // form is not quietly wrong in a corner of the workspace.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(109);

    int agreed = 0, attempted = 0;
    for (int trial = 0; trial < 1500; ++trial) {
        const JointVector q = random_configuration(model, rng, 0.8);
        const Transform target = forward_kinematics(model, q);

        JointVector seed = q;
        std::uniform_real_distribution<double> nudge(-0.15, 0.15);
        for (std::size_t i = 0; i < kNumJoints; ++i) seed[i] += nudge(rng);

        const IkResult analytic = inverse_kinematics(model, target, seed);
        const IkResult numerical = inverse_kinematics_numerical(model, target, seed);
        if (!analytic.ok() || !numerical.ok()) continue;
        ++attempted;

        double worst = 0.0;
        for (std::size_t i = 0; i < kNumJoints; ++i)
            worst = std::max(worst, std::fabs(wrap_angle(analytic.q[i] - numerical.q[i])));
        if (worst < 1e-6) ++agreed;
    }
    CHECK(attempted > 1000);
    // Both land on the same branch whenever the seed is close, which is the
    // case being tested. A handful of near-singular poses can still split.
    CHECK(static_cast<double>(agreed) / static_cast<double>(attempted) > 0.98);
}

TEST(the_numerical_solver_converges_from_a_neutral_seed) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(113);
    const JointVector neutral{};

    int converged = 0;
    const int trials = 500;
    for (int trial = 0; trial < trials; ++trial) {
        const JointVector q = random_configuration(model, rng, 0.75);
        const Transform target = forward_kinematics(model, q);
        const IkResult result = inverse_kinematics_numerical(model, target, neutral);
        if (result.ok()) {
            ++converged;
            CHECK_NEAR(pose_position_error(model, result.q, target), 0.0, 1e-8);
            CHECK_NEAR(pose_orientation_error(model, result.q, target), 0.0, 1e-8);
        }
    }
    // Damped least squares from a single fixed seed is not a global solver and
    // is not claimed to be. The analytic solver is what the controller runs;
    // this one is the fallback and the cross-check. Anything above four fifths
    // from the worst possible seed is doing its job.
    CHECK(static_cast<double>(converged) / static_cast<double>(trials) > 0.8);
}

TEST(the_numerical_solver_survives_the_elbow_singularity) {
    const ArmModel model = a1_arm();
    JointVector stretched{};
    stretched[2] = kPi / 2.0;
    stretched[4] = 0.6;
    const Transform target = forward_kinematics(model, stretched);

    JointVector seed = stretched;
    seed[2] -= 0.02;
    const IkResult result = inverse_kinematics_numerical(model, target, seed);
    // Convergence is not guaranteed at a rank-deficient Jacobian; not
    // diverging is. Damping is there so the step stays finite and the residual
    // does not grow.
    for (std::size_t i = 0; i < kNumJoints; ++i) CHECK(std::isfinite(result.q[i]));
    CHECK(result.position_error < 1e-3);
}

TEST(the_analytic_solver_handles_the_wrist_singularity_without_a_nan) {
    const ArmModel model = a1_arm();
    // q5 = 0 makes joints 4 and 6 collinear: their sum is determined, the split
    // is not. The solver has to pick one rather than divide by zero.
    JointVector flat{};
    flat[0] = 0.3; flat[1] = 0.4; flat[2] = -0.8; flat[3] = 0.9; flat[4] = 0.0; flat[5] = -0.2;
    const Transform target = forward_kinematics(model, flat);

    const IkResult result = inverse_kinematics(model, target, flat);
    CHECK(result.ok());
    for (std::size_t i = 0; i < kNumJoints; ++i) CHECK(std::isfinite(result.q[i]));
    CHECK_NEAR(pose_position_error(model, result.q, target), 0.0, 1e-11);
    CHECK_NEAR(pose_orientation_error(model, result.q, target), 0.0, 1e-8);
}

TEST(the_solver_refuses_a_chain_without_a_spherical_wrist) {
    // The closed form is only valid when the last three axes meet. Given a
    // model where they do not, it returns nothing rather than something wrong.
    ArmModel model = a1_arm();
    model.dh[4].d = 0.04;  // pull the wrist axes apart
    CHECK(!model.has_spherical_wrist());

    const Transform target = forward_kinematics(model, JointVector{});
    CHECK(inverse_kinematics_all(model, target, JointVector{}).empty());
    CHECK(inverse_kinematics(model, target, JointVector{}).status == IkStatus::kUnreachable);

    // The numerical solver has no such requirement and still finds it.
    const IkResult numerical = inverse_kinematics_numerical(model, target, JointVector{});
    CHECK(numerical.ok());
}

TEST_MAIN()
