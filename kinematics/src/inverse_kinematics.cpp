#include "arm_kinematics/inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace ak {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Moving the shoulder costs more than moving the wrist: it swings more mass,
// draws more current and is the joint a wearer notices. The seed distance the
// branch selector minimises is weighted to say so.
constexpr std::array<double, kNumJoints> kJointCost{{3.0, 3.0, 2.0, 1.0, 1.0, 0.5}};

// Solve the last three joints from the rotation the wrist has to produce.
// R_36 is the roll-pitch-roll product; docs/dh_parameters.md has the expansion
// these expressions are read off.
void solve_wrist(const Mat3& r36, bool flipped, double seed_q4, double& q4, double& q5,
                 double& q6) {
    const double c5 = std::max(-1.0, std::min(1.0, r36(2, 2)));
    const double s5_magnitude =
        std::sqrt(std::max(0.0, r36(0, 2) * r36(0, 2) + r36(1, 2) * r36(1, 2)));

    if (s5_magnitude < 1e-9) {
        // Wrist singularity: joints 4 and 6 are collinear and only their sum
        // (or difference) is determined. Hold joint 4 where it is and put the
        // whole rotation into joint 6, which is the lighter of the two.
        q4 = seed_q4;
        q5 = c5 >= 0.0 ? 0.0 : kPi;
        if (c5 >= 0.0) {
            q6 = wrap_angle(std::atan2(r36(1, 0), r36(0, 0)) - q4);
        } else {
            q6 = wrap_angle(q4 - std::atan2(-r36(1, 0), -r36(0, 0)));
        }
        return;
    }

    const double s5 = flipped ? -s5_magnitude : s5_magnitude;
    q5 = std::atan2(s5, c5);
    q4 = std::atan2(r36(1, 2) / s5, r36(0, 2) / s5);
    q6 = std::atan2(r36(2, 1) / s5, -r36(2, 0) / s5);
}

}  // namespace

double branch_seed_cost(const JointVector& candidate, const JointVector& seed) {
    double total = 0.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double d = wrap_angle(candidate[i] - seed[i]);
        total += kJointCost[i] * d * d;
    }
    return total;
}

std::vector<IkSolution> inverse_kinematics_all(const ArmModel& model, const Transform& pose,
                                               const JointVector& seed) {
    std::vector<IkSolution> solutions;
    if (!model.has_spherical_wrist()) return solutions;

    const double d1 = model.dh[0].d;
    const double a2 = model.dh[1].a;
    const double d4 = model.dh[3].d;
    const double d6 = model.dh[5].d;

    // Back off along the approach axis to the point where the three wrist axes
    // meet. That point depends only on the first three joints, which is what
    // makes the problem separable.
    const Vec3 pc = pose.translation - pose.axis(2) * d6;
    const double planar = std::sqrt(pc.x * pc.x + pc.y * pc.y);
    const double s = pc.z - d1;

    for (int shoulder = 0; shoulder < 2; ++shoulder) {
        const bool forward = shoulder == 0;
        double q1;
        if (planar < 1e-9) {
            // The wrist centre sits on the shoulder axis, so joint 1 does not
            // move it. Any value solves the position; keep the seed.
            q1 = seed[0];
        } else {
            q1 = forward ? std::atan2(pc.y, pc.x) : wrap_angle(std::atan2(pc.y, pc.x) + kPi);
        }
        const double r = forward ? planar : -planar;

        const double sin_q3 = (r * r + s * s - a2 * a2 - d4 * d4) / (2.0 * a2 * d4);
        if (sin_q3 > 1.0 + 1e-9 || sin_q3 < -1.0 - 1e-9) continue;
        const double sin_clamped = std::max(-1.0, std::min(1.0, sin_q3));
        const double cos_magnitude = std::sqrt(std::max(0.0, 1.0 - sin_clamped * sin_clamped));

        for (int elbow = 0; elbow < 2; ++elbow) {
            const bool up = elbow == 0;
            const double cos_q3 = up ? cos_magnitude : -cos_magnitude;
            const double q3 = std::atan2(sin_clamped, cos_q3);

            const double a = a2 + d4 * sin_clamped;
            const double b = d4 * cos_q3;
            const double det = a * a + b * b;
            if (det < 1e-18) continue;
            const double q2 = std::atan2(b * r + a * s, a * r - b * s);

            // With the arm placed, whatever rotation is left over belongs to
            // the wrist.
            const JointVector partial{{q1, q2, q3, 0.0, 0.0, 0.0}};
            const auto frames = link_transforms(model, partial);
            const Mat3 r36 = frames[2].rotation.transposed() * pose.rotation;

            for (int flip = 0; flip < 2; ++flip) {
                double q4 = 0.0, q5 = 0.0, q6 = 0.0;
                solve_wrist(r36, flip == 1, seed[3], q4, q5, q6);

                IkSolution sol;
                sol.q = {{wrap_angle(q1), wrap_angle(q2), wrap_angle(q3), wrap_angle(q4),
                          wrap_angle(q5), wrap_angle(q6)}};
                sol.branch = {forward, up, flip == 1};
                sol.within_limits = within_limits(model, sol.q, 1e-9);
                solutions.push_back(sol);
            }
        }
    }
    return solutions;
}

IkResult inverse_kinematics(const ArmModel& model, const Transform& pose,
                            const JointVector& seed) {
    IkResult result;
    const auto solutions = inverse_kinematics_all(model, pose, seed);
    if (solutions.empty()) {
        result.status = IkStatus::kUnreachable;
        return result;
    }

    const IkSolution* best = nullptr;
    double best_cost = 0.0;
    for (const auto& sol : solutions) {
        if (!sol.within_limits) continue;
        const double cost = branch_seed_cost(sol.q, seed);
        if (best == nullptr || cost < best_cost) {
            best = &sol;
            best_cost = cost;
        }
    }
    if (best == nullptr) {
        result.status = IkStatus::kOutsideLimits;
        return result;
    }

    result.q = best->q;
    result.branch = best->branch;
    result.status = IkStatus::kSuccess;

    const Transform achieved = forward_kinematics(model, result.q);
    result.position_error = norm(achieved.translation - pose.translation);
    result.orientation_error = norm(rotation_error(achieved.rotation, pose.rotation));
    return result;
}

IkResult inverse_kinematics_numerical(const ArmModel& model, const Transform& pose,
                                      const JointVector& seed,
                                      const NumericalIkOptions& options) {
    IkResult result;
    JointVector q = seed;
    if (options.respect_limits) clamp_to_limits(model, q);

    for (int iteration = 0; iteration <= options.max_iterations; ++iteration) {
        const Transform current = forward_kinematics(model, q);
        const Vec3 dp = pose.translation - current.translation;
        const Vec3 dr = rotation_error(current.rotation, pose.rotation);

        result.position_error = norm(dp);
        result.orientation_error = norm(dr);
        result.iterations = iteration;
        if (result.position_error < options.position_tolerance &&
            result.orientation_error < options.orientation_tolerance) {
            result.q = q;
            result.status = IkStatus::kSuccess;
            return result;
        }
        if (iteration == options.max_iterations) break;

        const auto j = jacobian(model, q);

        // Raise the damping as the arm approaches a singularity. Constant
        // damping is either too slow away from one or too jumpy at one; this
        // is the Levenberg idea tied to a measured conditioning number rather
        // than to the residual.
        const double sigma = manipulability(j);
        double lambda = options.damping;
        if (sigma < options.singularity_threshold) {
            const double ratio = sigma / options.singularity_threshold;
            lambda = options.damping + 0.05 * (1.0 - ratio) * (1.0 - ratio);
        }

        Vector<6> error;
        error(0, 0) = dp.x; error(1, 0) = dp.y; error(2, 0) = dp.z;
        error(3, 0) = dr.x; error(4, 0) = dr.y; error(5, 0) = dr.z;

        Vector<kNumJoints> step;
        if (!damped_least_squares(j, error, lambda, step)) {
            result.q = q;
            result.status = IkStatus::kNoConvergence;
            return result;
        }

        for (std::size_t i = 0; i < kNumJoints; ++i) {
            const double delta =
                std::max(-options.max_step, std::min(options.max_step, step(i, 0)));
            q[i] = wrap_angle(q[i] + delta);
        }
        if (options.respect_limits) clamp_to_limits(model, q);
    }

    result.q = q;
    result.status = IkStatus::kNoConvergence;
    return result;
}

}  // namespace ak
