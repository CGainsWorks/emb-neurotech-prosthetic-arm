// Sample the joint space and report what the arm can actually reach.
//
// The README quotes a reach envelope and an inverse-solution success rate.
// Both come from here, on a fixed seed, so anybody can rerun it and get the
// same numbers.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/inverse_kinematics.hpp"
#include "arm_kinematics/model.hpp"
#include "json.hpp"

namespace {

double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = fraction * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(position);
    const std::size_t upper = std::min(lower + 1, values.size() - 1);
    const double weight = position - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

}  // namespace

int main(int argc, char** argv) {
    int samples = 200000;
    unsigned long long seed = 20260822ULL;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (std::strcmp(argv[i], "--samples") == 0) samples = std::atoi(argv[i + 1]);
        if (std::strcmp(argv[i], "--seed") == 0) seed = std::strtoull(argv[i + 1], nullptr, 10);
    }

    const ak::ArmModel model = ak::a1_arm();
    std::mt19937_64 rng(seed);
    std::array<std::uniform_real_distribution<double>, ak::kNumJoints> joint_dist;
    for (std::size_t i = 0; i < ak::kNumJoints; ++i) {
        joint_dist[i] = std::uniform_real_distribution<double>(model.limits[i].lower,
                                                              model.limits[i].upper);
    }

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9, min_z = 1e9, max_z = -1e9;
    double min_reach = 1e9, max_reach = -1e9;
    std::vector<double> position_errors;
    std::vector<double> orientation_errors;
    position_errors.reserve(static_cast<std::size_t>(samples));
    orientation_errors.reserve(static_cast<std::size_t>(samples));

    int solved = 0;
    int outside_limits = 0;
    int unreachable = 0;
    long long branch_total = 0;

    for (int n = 0; n < samples; ++n) {
        ak::JointVector q{};
        for (std::size_t i = 0; i < ak::kNumJoints; ++i) q[i] = joint_dist[i](rng);

        const ak::Transform pose = ak::forward_kinematics(model, q);
        const ak::Vec3& p = pose.translation;
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
        const double reach = ak::norm(p);
        min_reach = std::min(min_reach, reach);
        max_reach = std::max(max_reach, reach);

        // Seed from the neutral pose rather than from the answer, so this
        // measures the solver rather than the seed.
        const ak::JointVector neutral{};
        const auto all = ak::inverse_kinematics_all(model, pose, neutral);
        for (const auto& sol : all) branch_total += sol.within_limits ? 1 : 0;

        const ak::IkResult solution = ak::inverse_kinematics(model, pose, neutral);
        if (solution.ok()) {
            ++solved;
            position_errors.push_back(solution.position_error);
            orientation_errors.push_back(solution.orientation_error);
        } else if (solution.status == ak::IkStatus::kOutsideLimits) {
            ++outside_limits;
        } else {
            ++unreachable;
        }
    }

    tools::Object envelope(2);
    envelope.field("x_m", tools::array({min_x, max_x}));
    envelope.field("y_m", tools::array({min_y, max_y}));
    envelope.field("z_m", tools::array({min_z, max_z}));
    envelope.number_field("min_reach_m", min_reach);
    envelope.number_field("max_reach_m", max_reach);
    // The kinematic maximum, straight out along the chain, which the sampler
    // will only approach rather than hit.
    envelope.number_field("kinematic_max_reach_m",
                          model.dh[0].d + model.dh[1].a + model.dh[3].d + model.dh[5].d);

    tools::Object accuracy(2);
    accuracy.number_field("position_median_m", percentile(position_errors, 0.5));
    accuracy.number_field("position_p99_m", percentile(position_errors, 0.99));
    accuracy.number_field("position_max_m",
                          position_errors.empty()
                              ? 0.0
                              : *std::max_element(position_errors.begin(), position_errors.end()));
    accuracy.number_field("orientation_median_rad", percentile(orientation_errors, 0.5));
    accuracy.number_field("orientation_p99_rad", percentile(orientation_errors, 0.99));
    accuracy.number_field(
        "orientation_max_rad",
        orientation_errors.empty()
            ? 0.0
            : *std::max_element(orientation_errors.begin(), orientation_errors.end()));

    tools::Object root;
    root.string_field("report", "workspace");
    root.string_field("source", "kinematics/tools/workspace_report.cpp");
    root.number_field("samples", samples);
    root.number_field("seed", static_cast<double>(seed));
    root.string_field("sampling", "uniform over the joint limits, so denser near the centre "
                                  "of the workspace than a uniform Cartesian sample would be");
    root.field("envelope", envelope.str());
    root.number_field("solved", solved);
    root.number_field("outside_limits", outside_limits);
    root.number_field("unreachable", unreachable);
    root.number_field("solved_fraction", static_cast<double>(solved) / static_cast<double>(samples));
    root.number_field("mean_in_limit_branches",
                      static_cast<double>(branch_total) / static_cast<double>(samples));
    root.field("round_trip_error", accuracy.str());

    std::printf("%s\n", root.str().c_str());
    return 0;
}
