#include "arm_kinematics/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace ak {
namespace {

// Fraction of the move spent accelerating in the trapezoidal profile. A third
// each way is the usual compromise: any smaller and the acceleration spike
// grows without shortening the move much, any larger and it degenerates into a
// triangle that never cruises.
constexpr double kAccelFraction = 1.0 / 3.0;

// Peak first and second derivative of the normalised profile, for a move of
// unit displacement over unit duration. Both are exact.
//   quintic     s(t) = 10t^3 - 15t^4 + 6t^5,  max s' = 15/8, max s'' = 10/sqrt(3)
//   trapezoid   with f = 1/3,                 max s' = 1/(1-f), max s'' = 1/(f(1-f))
struct ProfileShape {
    double peak_velocity;
    double peak_acceleration;
};

ProfileShape shape_of(ProfileType profile) {
    if (profile == ProfileType::kQuintic) return {1.875, 5.773502691896258};
    return {1.0 / (1.0 - kAccelFraction), 1.0 / (kAccelFraction * (1.0 - kAccelFraction))};
}

// The normalised profile and its derivatives with respect to time, given the
// total duration. tau runs 0 to 1.
void evaluate_profile(ProfileType profile, double tau, double duration, double& s, double& sd,
                      double& sdd) {
    if (profile == ProfileType::kQuintic) {
        const double t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
        s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
        sd = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / duration;
        sdd = (60.0 * tau - 180.0 * t2 + 120.0 * t3) / (duration * duration);
        return;
    }

    const double f = kAccelFraction;
    const double peak = 1.0 / (1.0 - f);              // cruise velocity, normalised
    const double accel = peak / f;                     // normalised acceleration
    if (tau < f) {
        s = 0.5 * accel * tau * tau;
        sd = accel * tau / duration;
        sdd = accel / (duration * duration);
    } else if (tau < 1.0 - f) {
        s = 0.5 * accel * f * f + peak * (tau - f);
        sd = peak / duration;
        sdd = 0.0;
    } else {
        const double remaining = 1.0 - tau;
        s = 1.0 - 0.5 * accel * remaining * remaining;
        sd = accel * remaining / duration;
        sdd = -accel / (duration * duration);
    }
}

}  // namespace

JointTrajectory::JointTrajectory(JointVector start, JointVector goal, double duration,
                                 ProfileType profile)
    : start_(start), goal_(goal), duration_(duration), profile_(profile) {
    for (std::size_t i = 0; i < kNumJoints; ++i) delta_[i] = goal_[i] - start_[i];
}

TrajectoryPoint JointTrajectory::sample(double time) const {
    TrajectoryPoint point;
    point.time = time;

    if (duration_ <= 0.0 || time >= duration_) {
        point.q = goal_;
        return point;
    }
    if (time <= 0.0) {
        point.q = start_;
        return point;
    }

    double s = 0.0, sd = 0.0, sdd = 0.0;
    evaluate_profile(profile_, time / duration_, duration_, s, sd, sdd);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        point.q[i] = start_[i] + delta_[i] * s;
        point.qd[i] = delta_[i] * sd;
        point.qdd[i] = delta_[i] * sdd;
    }
    return point;
}

std::vector<TrajectoryPoint> JointTrajectory::discretise(int steps) const {
    std::vector<TrajectoryPoint> points;
    if (steps < 1) return points;
    points.reserve(static_cast<std::size_t>(steps) + 1);
    for (int i = 0; i <= steps; ++i) {
        points.push_back(sample(duration_ * static_cast<double>(i) / static_cast<double>(steps)));
    }
    return points;
}

JointTrajectory plan_joint_move(const ArmModel& model, const JointVector& start,
                                const JointVector& goal, ProfileType profile,
                                const PlannerLimits& limits) {
    const ProfileShape shape = shape_of(profile);
    const double accel_ceiling = limits.max_acceleration * limits.acceleration_scale;

    // The move takes as long as the slowest joint needs and every joint then
    // rides the same normalised profile, so they arrive together.
    double duration = limits.min_duration;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double distance = std::fabs(goal[i] - start[i]);
        if (distance <= 0.0) continue;
        const double velocity_ceiling = model.limits[i].velocity * limits.velocity_scale;
        if (velocity_ceiling > 0.0) {
            duration = std::max(duration, shape.peak_velocity * distance / velocity_ceiling);
        }
        if (accel_ceiling > 0.0) {
            duration = std::max(duration, std::sqrt(shape.peak_acceleration * distance / accel_ceiling));
        }
    }
    return JointTrajectory(start, goal, duration, profile);
}

Transform interpolate_pose(const Transform& from, const Transform& to, double s) {
    Transform out;
    out.translation = from.translation + (to.translation - from.translation) * s;
    const Vec3 w = rotation_error(from.rotation, to.rotation);
    out.rotation = rotation_from_vector(w * s) * from.rotation;
    return out;
}

CartesianPlan plan_cartesian_line(const ArmModel& model, const JointVector& start_q,
                                  const Transform& goal_pose, double duration, int waypoints,
                                  ProfileType profile) {
    CartesianPlan plan;
    plan.duration = duration;
    if (waypoints < 2 || duration <= 0.0) {
        plan.status = IkStatus::kNoConvergence;
        return plan;
    }

    const Transform start_pose = forward_kinematics(model, start_q);
    JointVector seed = start_q;

    plan.points.reserve(static_cast<std::size_t>(waypoints));
    for (int i = 0; i < waypoints; ++i) {
        const double tau = static_cast<double>(i) / static_cast<double>(waypoints - 1);
        double s = 0.0, sd = 0.0, sdd = 0.0;
        evaluate_profile(profile, tau, duration, s, sd, sdd);

        const Transform target = interpolate_pose(start_pose, goal_pose, s);
        const IkResult solution = inverse_kinematics(model, target, seed);
        if (!solution.ok()) {
            plan.status = solution.status;
            plan.failed_at = i;
            return plan;
        }

        TrajectoryPoint point;
        point.time = tau * duration;
        point.q = solution.q;
        plan.points.push_back(point);
        seed = solution.q;
    }

    // Joint rates by central difference over the waypoint spacing. The path is
    // a straight line in space, not in joint space, so there is no closed form
    // to differentiate here.
    const double dt = duration / static_cast<double>(waypoints - 1);
    for (std::size_t i = 0; i < plan.points.size(); ++i) {
        for (std::size_t j = 0; j < kNumJoints; ++j) {
            double rate = 0.0;
            if (i == 0) {
                rate = (plan.points[1].q[j] - plan.points[0].q[j]) / dt;
            } else if (i + 1 == plan.points.size()) {
                rate = (plan.points[i].q[j] - plan.points[i - 1].q[j]) / dt;
            } else {
                rate = (plan.points[i + 1].q[j] - plan.points[i - 1].q[j]) / (2.0 * dt);
            }
            plan.points[i].qd[j] = rate;
            plan.max_joint_velocity = std::max(plan.max_joint_velocity, std::fabs(rate));
        }
    }

    plan.status = IkStatus::kSuccess;
    return plan;
}

}  // namespace ak
