#include <random>

#include "arm_kinematics/dynamics.hpp"
#include "test_framework.hpp"

using namespace ak;

namespace {

JointVector random_configuration(const ArmModel& model, std::mt19937_64& rng) {
    JointVector q{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        std::uniform_real_distribution<double> dist(model.limits[i].lower, model.limits[i].upper);
        q[i] = dist(rng);
    }
    return q;
}

JointVector random_rates(std::mt19937_64& rng, double magnitude) {
    JointVector qd{};
    std::uniform_real_distribution<double> dist(-magnitude, magnitude);
    for (std::size_t i = 0; i < kNumJoints; ++i) qd[i] = dist(rng);
    return qd;
}

}  // namespace

TEST(gravity_torque_is_the_gradient_of_potential_energy) {
    // The strongest check available on the recursion, because it compares it
    // against something derived a completely different way: the energy of the
    // chain, differentiated numerically. If a sign or a frame is wrong
    // anywhere in the Newton-Euler pass, these will not agree.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(201);
    const double h = 1e-6;

    for (int trial = 0; trial < 300; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector tau = gravity_torque(model, q);

        for (std::size_t i = 0; i < kNumJoints; ++i) {
            JointVector forward = q, backward = q;
            forward[i] += h;
            backward[i] -= h;
            const double gradient =
                (potential_energy(model, forward) - potential_energy(model, backward)) / (2.0 * h);
            CHECK_NEAR(tau[i], gradient, 1e-6);
        }
    }
}

TEST(the_last_joint_carries_no_gravity_load_about_its_own_axis) {
    // Joint 6 spins the terminal device about its own symmetry axis. A body
    // that is symmetric about the axis it turns on has no gravity moment about
    // it, whatever the rest of the arm is doing.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(202);
    for (int trial = 0; trial < 200; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector tau = gravity_torque(model, q);
        CHECK_NEAR(tau[5], 0.0, 1e-12);
    }
}

TEST(the_shoulder_holds_the_whole_arm_at_full_stretch) {
    // Arm horizontal and straight out, which is the worst case the actuator
    // sizing is done against. Compare against the elementary sum of moments.
    const ArmModel model = a1_arm();
    JointVector q{};
    q[2] = 1.5707963267948966;  // elbow straight, forearm in line with the upper arm

    const JointVector tau = gravity_torque(model, q);

    // Independent calculation: total weight times the horizontal distance from
    // the shoulder axis to the combined centre of mass.
    const auto frames = link_transforms(model, q);
    double mass = 0.0, moment = 0.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const Vec3 com = frames[i].apply(model.links[i].com);
        mass += model.links[i].mass;
        moment += model.links[i].mass * com.x;
    }
    const Vec3 tool_com = frames.back().apply(model.tool.com);
    mass += model.tool.mass;
    moment += model.tool.mass * tool_com.x;

    const double expected = mass * 9.80665 * (moment / mass);
    // Joint 2 is the shoulder pitch. Its sign convention has a positive angle
    // lifting the arm, so holding against gravity is a negative torque.
    CHECK_NEAR(std::fabs(tau[1]), expected, 1e-9);
    CHECK(std::fabs(tau[1]) > 3.0);
}

TEST(the_mass_matrix_is_symmetric) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(203);
    for (int trial = 0; trial < 500; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const auto m = mass_matrix(model, q);
        for (std::size_t r = 0; r < kNumJoints; ++r)
            for (std::size_t c = r + 1; c < kNumJoints; ++c)
                CHECK_NEAR(m(r, c), m(c, r), 1e-12);
    }
}

TEST(the_mass_matrix_is_positive_definite) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(204);
    for (int trial = 0; trial < 300; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const auto m = mass_matrix(model, q);
        for (int probe = 0; probe < 20; ++probe) {
            const JointVector v = random_rates(rng, 1.0);
            double quadratic = 0.0;
            for (std::size_t r = 0; r < kNumJoints; ++r)
                for (std::size_t c = 0; c < kNumJoints; ++c) quadratic += v[r] * m(r, c) * v[c];
            CHECK(quadratic > 0.0);
        }
    }
}

TEST(the_mass_matrix_reproduces_the_kinetic_energy) {
    // Kinetic energy computed from link velocities against the same quantity
    // computed as half qd transpose M qd. Two independent routes to one number.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(205);
    for (int trial = 0; trial < 300; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector qd = random_rates(rng, 2.0);
        const auto m = mass_matrix(model, q);

        double from_matrix = 0.0;
        for (std::size_t r = 0; r < kNumJoints; ++r)
            for (std::size_t c = 0; c < kNumJoints; ++c) from_matrix += qd[r] * m(r, c) * qd[c];
        from_matrix *= 0.5;

        CHECK_NEAR(from_matrix, kinetic_energy(model, q, qd), 1e-9);
    }
}

TEST(forward_and_inverse_dynamics_undo_each_other) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(206);
    for (int trial = 0; trial < 500; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector qd = random_rates(rng, 2.0);
        const JointVector tau = random_rates(rng, 5.0);

        JointVector qdd{};
        CHECK(forward_dynamics(model, q, qd, tau, qdd));
        const JointVector recovered = inverse_dynamics(model, q, qd, qdd, true);
        for (std::size_t i = 0; i < kNumJoints; ++i) CHECK_NEAR(recovered[i], tau[i], 1e-9);
    }
}

TEST(inverse_dynamics_splits_into_inertia_velocity_and_gravity) {
    // tau = M qdd + C(q, qd) + G(q) is the identity the joint controller relies
    // on when it feeds gravity forward, so it is worth pinning down.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(207);
    for (int trial = 0; trial < 300; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector qd = random_rates(rng, 2.0);
        const JointVector qdd = random_rates(rng, 3.0);

        const JointVector full = inverse_dynamics(model, q, qd, qdd, true);
        const auto m = mass_matrix(model, q);
        const JointVector velocity = velocity_torque(model, q, qd);
        const JointVector gravity = gravity_torque(model, q);

        for (std::size_t i = 0; i < kNumJoints; ++i) {
            double inertial = 0.0;
            for (std::size_t j = 0; j < kNumJoints; ++j) inertial += m(i, j) * qdd[j];
            CHECK_NEAR(full[i], inertial + velocity[i] + gravity[i], 1e-9);
        }
    }
}

TEST(velocity_torque_is_quadratic_in_the_joint_rates) {
    // Coriolis and centrifugal terms scale with the square of speed. Doubling
    // every rate has to quadruple them; anything else means a term has crept
    // in that does not belong.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(208);
    for (int trial = 0; trial < 200; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector qd = random_rates(rng, 1.5);
        JointVector doubled{};
        for (std::size_t i = 0; i < kNumJoints; ++i) doubled[i] = 2.0 * qd[i];

        const JointVector single = velocity_torque(model, q, qd);
        const JointVector quad = velocity_torque(model, q, doubled);
        for (std::size_t i = 0; i < kNumJoints; ++i) CHECK_NEAR(quad[i], 4.0 * single[i], 1e-9);
    }
}

TEST(a_stationary_arm_needs_no_velocity_torque) {
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(209);
    for (int trial = 0; trial < 100; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector tau = velocity_torque(model, q, JointVector{});
        for (std::size_t i = 0; i < kNumJoints; ++i) CHECK_NEAR(tau[i], 0.0, 1e-14);
    }
}

TEST(the_integrator_conserves_energy_to_first_order) {
    // No torque, no friction: the arm falls under gravity and total energy has
    // to stay put. Semi-implicit Euler does not conserve it exactly, so the
    // useful claim is not an absolute bound but the order of the error. Halving
    // the step has to halve the drift and if it does then the drift is the
    // integrator rather than a mistake in the dynamics: a wrong Coriolis term
    // would pump energy in at a rate that does not shrink with the step.
    const ArmModel model = a1_arm();

    struct Run {
        double drift;
        double peak_kinetic;
    };

    auto simulate = [&model](double dt) {
        JointVector q{};
        q[1] = 0.9;
        q[2] = -0.7;
        q[4] = 0.4;
        JointVector qd{};
        const double initial = potential_energy(model, q) + kinetic_energy(model, q, qd);
        Run run{0.0, 0.0};
        const int steps = static_cast<int>(1.0 / dt);
        for (int step = 0; step < steps; ++step) {
            if (!integrate(model, q, qd, JointVector{}, dt)) return Run{1e9, 1.0};
            const double kinetic = kinetic_energy(model, q, qd);
            const double total = potential_energy(model, q) + kinetic;
            run.drift = std::max(run.drift, std::fabs(total - initial));
            run.peak_kinetic = std::max(run.peak_kinetic, kinetic);
        }
        return run;
    };

    const Run milli = simulate(1e-3);       // the rate the simulated backend runs at
    const Run half = simulate(5e-4);
    const Run quarter = simulate(2.5e-4);

    // Something has to actually happen, or conservation is trivial.
    CHECK(milli.peak_kinetic > 1.0);

    // Under a tenth of the energy the arm is swinging around, at 1 kHz.
    CHECK(milli.drift < 0.10 * milli.peak_kinetic);

    // First order: each halving of the step halves the drift.
    CHECK(half.drift < milli.drift * 0.60);
    CHECK(half.drift > milli.drift * 0.40);
    CHECK(quarter.drift < half.drift * 0.60);
    CHECK(quarter.drift > half.drift * 0.40);
}

TEST(the_tool_payload_changes_the_gravity_load) {
    // The terminal device is folded into the last link rather than carried
    // separately, so this checks it actually got there.
    ArmModel light = a1_arm();
    light.tool.mass = 0.0;
    for (auto& value : light.tool.inertia) value = 0.0;

    const ArmModel loaded = a1_arm();
    JointVector q{};
    q[2] = 1.5707963267948966;

    const JointVector without = gravity_torque(light, q);
    const JointVector with = gravity_torque(loaded, q);
    CHECK(std::fabs(with[1]) > std::fabs(without[1]) + 0.5);
    CHECK_NEAR(with[5], without[5], 1e-12);
}

TEST(the_worst_case_gravity_load_fits_inside_the_actuator_ratings) {
    // The number that decides whether the arm can hold itself up. Sampled over
    // the whole joint space, so no configuration inside the stops asks an
    // actuator for more continuous torque than it is rated for.
    const ArmModel model = a1_arm();
    std::mt19937_64 rng(210);
    JointVector worst{};

    for (int trial = 0; trial < 200000; ++trial) {
        const JointVector q = random_configuration(model, rng);
        const JointVector tau = gravity_torque(model, q);
        for (std::size_t i = 0; i < kNumJoints; ++i)
            worst[i] = std::max(worst[i], std::fabs(tau[i]));
    }

    for (std::size_t i = 0; i < kNumJoints; ++i) CHECK(worst[i] <= model.limits[i].effort);
}

TEST_MAIN()
