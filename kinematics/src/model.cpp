#include "arm_kinematics/model.hpp"

#include <algorithm>
#include <cmath>

namespace ak {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;

// Link dimensions, metres. docs/dh_parameters.md records where each comes from.
constexpr double kD1 = 0.110;  // base plate to the shoulder pitch axis
constexpr double kA2 = 0.300;  // upper arm, shoulder to elbow
constexpr double kD4 = 0.260;  // forearm, elbow to wrist centre
constexpr double kD6 = 0.095;  // wrist centre to the terminal device reference

// Cylinder about its own axis and about a transverse axis through the centre.
constexpr double axial(double mass, double radius) { return 0.5 * mass * radius * radius; }
constexpr double transverse(double mass, double length, double radius) {
    return mass * (3.0 * radius * radius + length * length) / 12.0;
}

}  // namespace

ArmModel a1_arm() {
    ArmModel m;

    m.dh = {{
        {0.0, kD1, 0.0, kHalfPi},   // 1 shoulder rotation
        {0.0, 0.0, kA2, 0.0},       // 2 shoulder flexion / extension
        {0.0, 0.0, 0.0, kHalfPi},   // 3 elbow flexion
        {0.0, kD4, 0.0, -kHalfPi},  // 4 forearm pronation / supination
        {0.0, 0.0, 0.0, kHalfPi},   // 5 wrist flexion / extension
        {0.0, kD6, 0.0, 0.0},       // 6 terminal device roll
    }};

    m.joint_names = {{"shoulder_rotation", "shoulder_flexion", "elbow_flexion",
                      "forearm_rotation", "wrist_flexion", "tool_roll"}};

    // Ranges are the mechanical stops, not a clinical range of motion. The
    // elbow is symmetric because this is a robot arm on a bench mount and both
    // the elbow-up and elbow-down branches of the inverse solution are
    // reachable; a socket-mounted build would fit a one-sided stop.
    // Effort is the actuator's rated continuous torque at the joint output,
    // after the reduction quoted in hardware/semg-servo-board/docs/thermal_budget.md.
    m.limits = {{
        {-1.75, 1.75, 2.50, 8.0},
        {-2.09, 2.09, 2.50, 8.0},
        {-2.53, 2.53, 2.50, 6.0},
        {-3.05, 3.05, 4.00, 2.5},
        {-1.75, 1.75, 4.00, 2.5},
        {-3.05, 3.05, 4.00, 1.2},
    }};

    // Frame i sits at the distal end of link i in the standard convention, so
    // a link's centre of mass is generally at a negative offset along the axis
    // it extends. Masses are the printed structure plus the actuator that
    // drives the next joint.
    LinkInertia l1;
    l1.mass = 0.35;
    l1.com = {0.0, 0.0, 0.020};
    l1.inertia = {transverse(0.35, 0.09, 0.040), transverse(0.35, 0.09, 0.040),
                  axial(0.35, 0.040), 0.0, 0.0, 0.0};

    LinkInertia l2;
    l2.mass = 0.55;
    l2.com = {-0.150, 0.0, 0.0};
    l2.inertia = {axial(0.55, 0.035), transverse(0.55, kA2, 0.035),
                  transverse(0.55, kA2, 0.035), 0.0, 0.0, 0.0};

    LinkInertia l3;
    l3.mass = 0.25;
    l3.com = {0.0, 0.0, 0.030};
    l3.inertia = {transverse(0.25, 0.07, 0.035), transverse(0.25, 0.07, 0.035),
                  axial(0.25, 0.035), 0.0, 0.0, 0.0};

    LinkInertia l4;
    l4.mass = 0.45;
    l4.com = {0.0, 0.0, -0.120};
    l4.inertia = {transverse(0.45, kD4, 0.032), transverse(0.45, kD4, 0.032),
                  axial(0.45, 0.032), 0.0, 0.0, 0.0};

    LinkInertia l5;
    l5.mass = 0.12;
    l5.com = {0.0, 0.0, 0.0};
    l5.inertia = {transverse(0.12, 0.05, 0.028), transverse(0.12, 0.05, 0.028),
                  axial(0.12, 0.028), 0.0, 0.0, 0.0};

    LinkInertia l6;
    l6.mass = 0.08;
    l6.com = {0.0, 0.0, -0.030};
    l6.inertia = {transverse(0.08, 0.04, 0.025), transverse(0.08, 0.04, 0.025),
                  axial(0.08, 0.025), 0.0, 0.0, 0.0};

    m.links = {{l1, l2, l3, l4, l5, l6}};

    // A three-digit terminal device. Modelled as a rigid payload: the finger
    // degrees of freedom are driven open loop from the gesture class and are
    // not part of the six-axis kinematic chain.
    m.tool.mass = 0.40;
    m.tool.com = {0.0, 0.0, 0.035};
    m.tool.inertia = {transverse(0.40, 0.11, 0.035), transverse(0.40, 0.11, 0.035),
                      axial(0.40, 0.035), 0.0, 0.0, 0.0};

    return m;
}

bool clamp_to_limits(const ArmModel& model, JointVector& q) {
    bool untouched = true;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double clamped = std::max(model.limits[i].lower, std::min(model.limits[i].upper, q[i]));
        if (clamped != q[i]) untouched = false;
        q[i] = clamped;
    }
    return untouched;
}

bool within_limits(const ArmModel& model, const JointVector& q, double tolerance) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        if (q[i] < model.limits[i].lower - tolerance) return false;
        if (q[i] > model.limits[i].upper + tolerance) return false;
    }
    return true;
}

}  // namespace ak
