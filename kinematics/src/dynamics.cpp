#include "arm_kinematics/dynamics.hpp"

#include <array>
#include <cmath>

namespace ak {
namespace {

// The tool is rigidly bolted to the flange, so it is folded into link 6 rather
// than carried as a seventh body: same answer, one less special case in the
// recursion. Parallel axis theorem, both inertias taken about the combined
// centre of mass.
LinkInertia flange_with_tool(const ArmModel& model) {
    const LinkInertia& link = model.links[kNumJoints - 1];
    const LinkInertia& tool = model.tool;
    const double mass = link.mass + tool.mass;
    if (mass <= 0.0) return link;

    LinkInertia merged;
    merged.mass = mass;
    merged.com = (link.com * link.mass + tool.com * tool.mass) * (1.0 / mass);

    const Mat3 link_i = inertia_tensor(link);
    const Mat3 tool_i = inertia_tensor(tool);
    const Vec3 dl = link.com - merged.com;
    const Vec3 dt = tool.com - merged.com;

    // I_about_new = I_about_own + m (|d|^2 E - d d^T)
    auto shifted = [](const Mat3& inertia, double m, const Vec3& d) {
        Mat3 out = inertia;
        const double dd = dot(d, d);
        for (std::size_t r = 0; r < 3; ++r)
            for (std::size_t c = 0; c < 3; ++c)
                out(r, c) += m * ((r == c ? dd : 0.0) - d[r] * d[c]);
        return out;
    };

    const Mat3 total = shifted(link_i, link.mass, dl) + shifted(tool_i, tool.mass, dt);
    merged.inertia = {total(0, 0), total(1, 1), total(2, 2),
                      total(0, 1), total(0, 2), total(1, 2)};
    return merged;
}

std::array<LinkInertia, kNumJoints> bodies(const ArmModel& model) {
    std::array<LinkInertia, kNumJoints> out = model.links;
    out[kNumJoints - 1] = flange_with_tool(model);
    return out;
}

}  // namespace

Mat3 inertia_tensor(const LinkInertia& link) {
    Mat3 m;
    m(0, 0) = link.inertia[0];
    m(1, 1) = link.inertia[1];
    m(2, 2) = link.inertia[2];
    m(0, 1) = m(1, 0) = link.inertia[3];
    m(0, 2) = m(2, 0) = link.inertia[4];
    m(1, 2) = m(2, 1) = link.inertia[5];
    return m;
}

JointVector inverse_dynamics(const ArmModel& model, const JointVector& q, const JointVector& qd,
                             const JointVector& qdd, bool include_gravity) {
    const auto body = bodies(model);

    // Rotation from frame i to frame i-1 and the vector from origin i-1 to
    // origin i expressed in frame i. Both come straight off the DH transform.
    std::array<Mat3, kNumJoints> r_prev_to_this;   // ^i R_{i-1}
    std::array<Vec3, kNumJoints> p_prev_to_this;   // ^i p_{i-1,i}
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const Transform t = dh_transform(model.dh[i], q[i]);
        r_prev_to_this[i] = t.rotation.transposed();
        p_prev_to_this[i] = r_prev_to_this[i] * t.translation;
    }

    // Base state. Carrying -g as a base acceleration is the standard way to
    // fold weight into the same recursion that handles inertia.
    Vec3 omega{}, alpha{}, accel{};
    if (include_gravity) accel = model.gravity * -1.0;

    std::array<Vec3, kNumJoints> omega_i, alpha_i, accel_com;
    const Vec3 z{0.0, 0.0, 1.0};

    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const Mat3& r = r_prev_to_this[i];
        const Vec3& p = p_prev_to_this[i];

        const Vec3 new_omega = r * (omega + z * qd[i]);
        const Vec3 new_alpha = r * (alpha + z * qdd[i] + cross(omega, z * qd[i]));
        const Vec3 new_accel =
            r * accel + cross(new_alpha, p) + cross(new_omega, cross(new_omega, p));

        omega_i[i] = new_omega;
        alpha_i[i] = new_alpha;
        accel_com[i] = new_accel + cross(new_alpha, body[i].com) +
                       cross(new_omega, cross(new_omega, body[i].com));

        omega = new_omega;
        alpha = new_alpha;
        accel = new_accel;
    }

    JointVector tau{};
    Vec3 force{}, moment{};
    for (std::size_t idx = kNumJoints; idx-- > 0;) {
        // Whatever the next link pushes back with, brought into this frame.
        Vec3 child_force{}, child_moment{};
        if (idx + 1 < kNumJoints) {
            const Mat3 r_child = r_prev_to_this[idx + 1].transposed();  // ^i R_{i+1}
            child_force = r_child * force;
            child_moment = r_child * moment;
        }

        const Mat3 inertia = inertia_tensor(body[idx]);
        const Vec3 this_force = child_force + accel_com[idx] * body[idx].mass;
        const Vec3 com_from_prev = p_prev_to_this[idx] + body[idx].com;

        const Vec3 this_moment = child_moment - cross(this_force, com_from_prev) +
                                 cross(child_force, body[idx].com) + inertia * alpha_i[idx] +
                                 cross(omega_i[idx], inertia * omega_i[idx]);

        // The joint axis is z of the previous frame, seen from this one.
        const Vec3 axis = r_prev_to_this[idx] * z;
        tau[idx] = dot(this_moment, axis);

        force = this_force;
        moment = this_moment;
    }
    return tau;
}

JointVector gravity_torque(const ArmModel& model, const JointVector& q) {
    const JointVector zero{};
    return inverse_dynamics(model, q, zero, zero, true);
}

JointVector velocity_torque(const ArmModel& model, const JointVector& q, const JointVector& qd) {
    const JointVector zero{};
    return inverse_dynamics(model, q, qd, zero, false);
}

Matrix<kNumJoints, kNumJoints> mass_matrix(const ArmModel& model, const JointVector& q) {
    Matrix<kNumJoints, kNumJoints> m;
    const JointVector zero{};
    for (std::size_t j = 0; j < kNumJoints; ++j) {
        JointVector unit{};
        unit[j] = 1.0;
        const JointVector column = inverse_dynamics(model, q, zero, unit, false);
        for (std::size_t i = 0; i < kNumJoints; ++i) m(i, j) = column[i];
    }
    return m;
}

bool forward_dynamics(const ArmModel& model, const JointVector& q, const JointVector& qd,
                      const JointVector& tau, JointVector& qdd) {
    const Matrix<kNumJoints, kNumJoints> m = mass_matrix(model, q);
    const JointVector bias = inverse_dynamics(model, q, qd, JointVector{}, true);

    Vector<kNumJoints> rhs;
    for (std::size_t i = 0; i < kNumJoints; ++i) rhs(i, 0) = tau[i] - bias[i];

    Vector<kNumJoints> solution;
    if (!solve<kNumJoints, 1>(m, rhs, solution)) return false;
    qdd = to_array<kNumJoints>(solution);
    return true;
}

double potential_energy(const ArmModel& model, const JointVector& q) {
    const auto body = bodies(model);
    const auto frames = link_transforms(model, q);
    double energy = 0.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const Vec3 com_world = frames[i].apply(body[i].com);
        energy -= body[i].mass * dot(model.gravity, com_world);
    }
    return energy;
}

double kinetic_energy(const ArmModel& model, const JointVector& q, const JointVector& qd) {
    const auto body = bodies(model);
    const auto frames = link_transforms(model, q);

    // Velocity of each link's centre of mass, accumulated joint by joint. The
    // Jacobian of a point on link i only involves joints up to i.
    double energy = 0.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        const Vec3 com_world = frames[i].apply(body[i].com);

        Vec3 linear{}, angular{};
        Transform previous = Transform::identity();
        for (std::size_t j = 0; j <= i; ++j) {
            const Vec3 axis = previous.axis(2);
            linear = linear + cross(axis, com_world - previous.translation) * qd[j];
            angular = angular + axis * qd[j];
            previous = frames[j];
        }

        const Mat3 inertia_world =
            frames[i].rotation * inertia_tensor(body[i]) * frames[i].rotation.transposed();
        energy += 0.5 * body[i].mass * dot(linear, linear);
        energy += 0.5 * dot(angular, inertia_world * angular);
    }
    return energy;
}

bool integrate(const ArmModel& model, JointVector& q, JointVector& qd, const JointVector& tau,
               double dt) {
    JointVector qdd{};
    if (!forward_dynamics(model, q, qd, tau, qdd)) return false;
    // Semi-implicit Euler: velocity first, then position from the new velocity.
    // At the 1 kHz the simulated backend runs it is stable where explicit Euler
    // is not and it costs nothing extra.
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        qd[i] += qdd[i] * dt;
        q[i] += qd[i] * dt;
    }
    return true;
}

}  // namespace ak
