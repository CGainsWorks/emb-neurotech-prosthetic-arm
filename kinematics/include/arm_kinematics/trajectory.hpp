// Time-parameterised trajectory generation.
//
// Two profiles, both synchronised so every joint starts and stops together.
// A move where the wrist arrives first and then the elbow catches up traces a
// curve nobody asked for and on a limb that reads as a twitch.
#pragma once

#include <vector>

#include "arm_kinematics/inverse_kinematics.hpp"
#include "arm_kinematics/model.hpp"

namespace ak {

enum class ProfileType {
    // Fifth-order polynomial in the path parameter. Continuous acceleration,
    // zero jerk at the ends. The default: it is what a servo geartrain likes
    // and what a wearer reads as deliberate rather than abrupt.
    kQuintic,
    // Constant-acceleration trapezoid. Reaches a goal in less time for the
    // same limits, at the cost of a step in acceleration at each corner.
    kTrapezoidal,
};

struct TrajectoryPoint {
    double time{0.0};   // s from the start of the move
    JointVector q{};
    JointVector qd{};
    JointVector qdd{};
};

// A synchronised point-to-point move. `duration` is the whole move; sampling
// past it returns the goal at rest, which is what a controller wants when it
// overruns by a tick.
class JointTrajectory {
  public:
    JointTrajectory() = default;
    JointTrajectory(JointVector start, JointVector goal, double duration, ProfileType profile);

    TrajectoryPoint sample(double time) const;
    double duration() const { return duration_; }
    const JointVector& start() const { return start_; }
    const JointVector& goal() const { return goal_; }
    ProfileType profile() const { return profile_; }
    bool empty() const { return duration_ <= 0.0; }

    // Uniformly spaced samples, endpoints included. `steps` is the number of
    // intervals, so the result has steps + 1 points.
    std::vector<TrajectoryPoint> discretise(int steps) const;

  private:
    JointVector start_{};
    JointVector goal_{};
    JointVector delta_{};
    double duration_{0.0};
    ProfileType profile_{ProfileType::kQuintic};
};

struct PlannerLimits {
    // Fractions of the model's joint limits. Below one so a plan has somewhere
    // to go when the controller adds a correction on top.
    double velocity_scale{0.6};
    double acceleration_scale{0.5};
    // Acceleration is not in the model, because a servo's rated torque and its
    // achievable acceleration are different questions. This is the ceiling
    // used for planning, before the scale above.
    double max_acceleration{6.0};  // rad/s^2
    double min_duration{0.15};     // s, so a tiny move is still not a step
};

// Point-to-point in joint space, synchronised, obeying the scaled limits.
JointTrajectory plan_joint_move(const ArmModel& model, const JointVector& start,
                                const JointVector& goal,
                                ProfileType profile = ProfileType::kQuintic,
                                const PlannerLimits& limits = {});

struct CartesianPlan {
    std::vector<TrajectoryPoint> points;
    double duration{0.0};
    IkStatus status{IkStatus::kSuccess};
    // Index of the waypoint the inverse solver first failed on; -1 when it did
    // not. Reported rather than swallowed so a caller can say which part of the
    // path was unreachable.
    int failed_at{-1};
    double max_joint_velocity{0.0};  // rad/s, over the whole path
};

// A straight line in space with the orientation carried along the shortest
// rotation, solved joint by joint. The path parameter follows the same profile
// as a joint move, so start and end are at rest.
//
// Every waypoint is seeded from the previous solution, which is what keeps the
// arm on one branch instead of flipping the elbow halfway along.
CartesianPlan plan_cartesian_line(const ArmModel& model, const JointVector& start_q,
                                  const Transform& goal_pose, double duration, int waypoints = 50,
                                  ProfileType profile = ProfileType::kQuintic);

// The shortest interpolation from `from` to `to` at parameter s in [0, 1].
Transform interpolate_pose(const Transform& from, const Transform& to, double s);

}  // namespace ak
