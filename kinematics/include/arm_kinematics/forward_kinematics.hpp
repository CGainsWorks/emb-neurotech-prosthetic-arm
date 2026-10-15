// Forward kinematics and the geometric Jacobian.
#pragma once

#include <array>

#include "arm_kinematics/model.hpp"
#include "arm_kinematics/transform.hpp"

namespace ak {

// The transform from frame i-1 to frame i for a single DH row at joint angle q.
Transform dh_transform(const DhRow& row, double q);

// Pose of every frame, base to flange. Element i is the transform from the base
// to frame i+1, so `.back()` is the tool pose.
std::array<Transform, kNumJoints> link_transforms(const ArmModel& model, const JointVector& q);

// Tool pose in the base frame.
Transform forward_kinematics(const ArmModel& model, const JointVector& q);

// Position of the wrist centre, the point where the three wrist axes meet.
// The analytic inverse solution is built around this point, so it is worth
// having on its own.
Vec3 wrist_centre(const ArmModel& model, const JointVector& q);

// Geometric Jacobian in the base frame: rows 0-2 are linear velocity of the
// tool origin, rows 3-5 are angular velocity, both per unit joint rate.
Matrix<6, kNumJoints> jacobian(const ArmModel& model, const JointVector& q);

// Smallest singular value of the Jacobian, obtained from the eigenvalues of
// J J^T by cyclic Jacobi. Zero at a singularity; the controller uses it to
// decide when to damp.
double manipulability(const Matrix<6, kNumJoints>& j);

}  // namespace ak
