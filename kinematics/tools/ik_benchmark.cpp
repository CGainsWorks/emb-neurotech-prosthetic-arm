// How long the kinematics costs, on this machine.
//
// These are host measurements on a desktop CPU, not measurements on the arm's
// controller and the JSON says so on every run. docs/CONTROL_LOOP_TIMING.md
// explains what they do and do not imply about the target.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "arm_kinematics/dynamics.hpp"
#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/inverse_kinematics.hpp"
#include "arm_kinematics/model.hpp"
#include "arm_kinematics/trajectory.hpp"
#include "json.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = fraction * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(position);
    const std::size_t upper = std::min(lower + 1, values.size() - 1);
    const double weight = position - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

std::string timing_object(const std::vector<double>& micros) {
    tools::Object o(2);
    o.number_field("iterations", static_cast<double>(micros.size()));
    o.number_field("median_us", percentile(micros, 0.5));
    o.number_field("p99_us", percentile(micros, 0.99));
    o.number_field("max_us", micros.empty() ? 0.0 : *std::max_element(micros.begin(), micros.end()));
    return o.str();
}

}  // namespace

int main(int argc, char** argv) {
    int iterations = 20000;
    int warmup = 2000;
    std::string label = "host";
    for (int i = 1; i + 1 < argc; i += 2) {
        if (std::strcmp(argv[i], "--iterations") == 0) iterations = std::atoi(argv[i + 1]);
        if (std::strcmp(argv[i], "--warmup") == 0) warmup = std::atoi(argv[i + 1]);
        if (std::strcmp(argv[i], "--label") == 0) label = argv[i + 1];
    }

    const ak::ArmModel model = ak::a1_arm();
    std::mt19937_64 rng(20260822ULL);
    std::array<std::uniform_real_distribution<double>, ak::kNumJoints> joint_dist;
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        // Inside the stops by a margin, so the sample is a pose the controller
        // would really be asked for rather than one hard against a limit.
        const double lower = model.limits[i].lower * 0.85;
        const double upper = model.limits[i].upper * 0.85;
        joint_dist[i] = std::uniform_real_distribution<double>(lower, upper);
    }

    std::vector<ak::JointVector> configurations;
    std::vector<ak::Transform> poses;
    configurations.reserve(static_cast<std::size_t>(iterations + warmup));
    poses.reserve(static_cast<std::size_t>(iterations + warmup));
    for (int n = 0; n < iterations + warmup; ++n) {
        ak::JointVector q{};
        for (std::size_t i = 0; i < ak::kNumJoints; ++i) q[i] = joint_dist[i](rng);
        configurations.push_back(q);
        poses.push_back(ak::forward_kinematics(model, q));
    }

    std::vector<double> fk, ik_analytic, ik_numerical, id, fd, jac, plan;
    fk.reserve(static_cast<std::size_t>(iterations));
    ik_analytic.reserve(static_cast<std::size_t>(iterations));
    ik_numerical.reserve(static_cast<std::size_t>(iterations));
    id.reserve(static_cast<std::size_t>(iterations));
    fd.reserve(static_cast<std::size_t>(iterations));
    jac.reserve(static_cast<std::size_t>(iterations));
    plan.reserve(static_cast<std::size_t>(iterations));

    // A sink the optimiser cannot prove is unused, so none of the calls below
    // gets deleted for having no observable effect.
    double sink = 0.0;
    int numerical_converged = 0;
    long long numerical_iterations = 0;

    const ak::JointVector neutral{};
    const ak::JointVector velocity{{0.4, -0.3, 0.5, 0.2, -0.6, 0.1}};

    for (int n = 0; n < iterations + warmup; ++n) {
        const bool timed = n >= warmup;
        const ak::JointVector& q = configurations[static_cast<std::size_t>(n)];
        const ak::Transform& pose = poses[static_cast<std::size_t>(n)];

        auto t0 = Clock::now();
        const ak::Transform out = ak::forward_kinematics(model, q);
        auto t1 = Clock::now();
        sink += out.translation.x;
        if (timed) fk.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

        t0 = Clock::now();
        const ak::IkResult analytic = ak::inverse_kinematics(model, pose, neutral);
        t1 = Clock::now();
        sink += analytic.q[0];
        if (timed) ik_analytic.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

        t0 = Clock::now();
        const ak::IkResult numerical = ak::inverse_kinematics_numerical(model, pose, neutral);
        t1 = Clock::now();
        sink += numerical.q[0];
        if (timed) {
            ik_numerical.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            if (numerical.ok()) ++numerical_converged;
            numerical_iterations += numerical.iterations;
        }

        t0 = Clock::now();
        const ak::JointVector torque = ak::inverse_dynamics(model, q, velocity, velocity);
        t1 = Clock::now();
        sink += torque[0];
        if (timed) id.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

        t0 = Clock::now();
        ak::JointVector qdd{};
        const bool ok = ak::forward_dynamics(model, q, velocity, torque, qdd);
        t1 = Clock::now();
        sink += ok ? qdd[0] : 0.0;
        if (timed) fd.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

        t0 = Clock::now();
        const auto j = ak::jacobian(model, q);
        t1 = Clock::now();
        sink += j(0, 0);
        if (timed) jac.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

        t0 = Clock::now();
        const ak::JointTrajectory trajectory = ak::plan_joint_move(model, neutral, q);
        const ak::TrajectoryPoint mid = trajectory.sample(trajectory.duration() * 0.5);
        t1 = Clock::now();
        sink += mid.q[0];
        if (timed) plan.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }

    tools::Object root;
    root.string_field("report", "kinematics latency");
    root.string_field("source", "kinematics/tools/ik_benchmark.cpp");
    root.string_field("label", label);
    root.bool_field("measured", true);
    root.string_field("disclaimer",
                      "Measured on the host that ran it, not on the arm controller. "
                      "See docs/CONTROL_LOOP_TIMING.md.");
    root.number_field("iterations", iterations);
    root.number_field("warmup", warmup);
    root.field("forward_kinematics", timing_object(fk));
    root.field("jacobian", timing_object(jac));
    root.field("inverse_kinematics_analytic", timing_object(ik_analytic));
    root.field("inverse_kinematics_numerical", timing_object(ik_numerical));
    root.field("inverse_dynamics", timing_object(id));
    root.field("forward_dynamics", timing_object(fd));
    root.field("trajectory_plan_and_sample", timing_object(plan));
    root.number_field("numerical_converged_fraction",
                      static_cast<double>(numerical_converged) / static_cast<double>(iterations));
    root.number_field("numerical_mean_iterations",
                      static_cast<double>(numerical_iterations) / static_cast<double>(iterations));
    // Printed so the compiler has to keep every call above.
    root.number_field("checksum", sink);

    std::printf("%s\n", root.str().c_str());
    return 0;
}
