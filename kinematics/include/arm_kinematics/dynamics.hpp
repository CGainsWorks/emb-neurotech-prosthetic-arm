// Rigid-body dynamics: inverse dynamics by recursive Newton-Euler, the joint
// space mass matrix and forward dynamics.
//
// The gravity torques this produces are what size the actuators and they are
// what the joint controller feeds forward so the position loop is not left
// fighting the arm's own weight. hardware/semg-servo-board/docs/thermal_budget.md
// consumes the worst-case numbers directly.
#pragma once

#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/model.hpp"

namespace ak {

// Inertia tensor of a link about its own centre of mass, in that link frame.
Mat3 inertia_tensor(const LinkInertia& link);

// Inverse dynamics: the joint torques that produce (qdd) at state (q, qd).
// With `include_gravity` false the result is the torque needed in free fall,
// which is what the mass matrix and the Coriolis term are built from.
JointVector inverse_dynamics(const ArmModel& model, const JointVector& q, const JointVector& qd,
                             const JointVector& qdd, bool include_gravity = true);

// The torque each joint must hold to keep the arm still against gravity.
JointVector gravity_torque(const ArmModel& model, const JointVector& q);

// Coriolis and centrifugal torques at (q, qd); zero when the arm is stationary.
JointVector velocity_torque(const ArmModel& model, const JointVector& q, const JointVector& qd);

// Joint space mass matrix, built column by column from the recursion. Symmetric
// and positive definite for any configuration of a serial chain with positive
// link masses, both of which the tests check.
Matrix<kNumJoints, kNumJoints> mass_matrix(const ArmModel& model, const JointVector& q);

// Forward dynamics: the acceleration a set of joint torques produces.
// Returns false only if the mass matrix is singular, which cannot happen for a
// physically realisable model and therefore signals a corrupt one.
bool forward_dynamics(const ArmModel& model, const JointVector& q, const JointVector& qd,
                      const JointVector& tau, JointVector& qdd);

// Gravitational potential energy of the whole chain, tool included. The
// gradient of this with respect to q is the gravity torque, which is how the
// tests check the recursion without reimplementing it.
double potential_energy(const ArmModel& model, const JointVector& q);

// Kinetic energy at (q, qd), computed from link velocities rather than from
// the mass matrix, so it is an independent check on it.
double kinetic_energy(const ArmModel& model, const JointVector& q, const JointVector& qd);

// One semi-implicit Euler step of the free arm under `tau`. Used by the
// simulated firmware backend in ros2_ws/src/arm_interface.
bool integrate(const ArmModel& model, JointVector& q, JointVector& qd, const JointVector& tau,
               double dt);

}  // namespace ak
