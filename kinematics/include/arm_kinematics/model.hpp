// The arm's geometry, joint limits and inertial parameters, in one place.
//
// This file is the single source of truth for the mechanism. The URDF in
// ros2_ws/src/arm_description is generated from it by
// scripts/generate_urdf.py and tools/dump_model.cpp emits the same numbers as
// JSON so a test can prove the two have not drifted apart. Change a link
// length here and everything downstream follows; change it in the URDF and CI
// fails.
#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "arm_kinematics/transform.hpp"

namespace ak {

inline constexpr std::size_t kNumJoints = 6;

using JointVector = std::array<double, kNumJoints>;

// One row of the standard (distal) Denavit-Hartenberg table.
//
// Standard rather than modified DH because the analytic solver below is the
// textbook anthropomorphic-arm decomposition, which is written in this
// convention; translating it would add a transcription risk for no gain.
struct DhRow {
    double theta_offset;  // rad, added to the joint variable
    double d;             // m, offset along the previous z
    double a;             // m, link length along the common normal
    double alpha;         // rad, twist about the common normal
};

struct JointLimits {
    double lower;      // rad
    double upper;      // rad
    double velocity;   // rad/s
    double effort;     // N m, the actuator's rated continuous torque at the joint
};

// Inertial parameters of link i, expressed in link frame i.
struct LinkInertia {
    double mass{0.0};             // kg
    Vec3 com{};                   // m, centre of mass in frame i
    std::array<double, 6> inertia{};  // kg m^2 about the centre of mass:
                                      // ixx, iyy, izz, ixy, ixz, iyz
};

struct ArmModel {
    std::array<DhRow, kNumJoints> dh{};
    std::array<JointLimits, kNumJoints> limits{};
    std::array<LinkInertia, kNumJoints> links{};
    std::array<std::string, kNumJoints> joint_names{};

    // Payload rigidly attached to the flange: the terminal device, expressed
    // in frame 6.
    LinkInertia tool{};

    Vec3 gravity{0.0, 0.0, -9.80665};

    bool has_spherical_wrist() const {
        // Joints 4, 5 and 6 intersect at a point exactly when a4 == a5 == 0 and
        // d5 == 0. The analytic solver requires it; the numerical one does not.
        return dh[3].a == 0.0 && dh[4].a == 0.0 && dh[4].d == 0.0;
    }
};

// The A1 six-axis research arm.
//
// An anthropomorphic shoulder-elbow chain closed by a roll-pitch-roll wrist.
// The three wrist axes are made concurrent on purpose: it costs a little
// anatomical fidelity, because a human wrist's flexion and deviation axes do
// not intersect and it buys a closed-form inverse solution that runs in about
// a microsecond and returns every branch. docs/dh_parameters.md derives the
// table and records where the dimensions come from.
ArmModel a1_arm();

// Clamp to the joint limits. Returns true when nothing had to be clamped.
bool clamp_to_limits(const ArmModel& model, JointVector& q);

bool within_limits(const ArmModel& model, const JointVector& q, double tolerance = 1e-9);

}  // namespace ak
