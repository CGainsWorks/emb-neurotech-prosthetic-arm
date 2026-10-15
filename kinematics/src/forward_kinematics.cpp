#include "arm_kinematics/forward_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace ak {

Transform dh_transform(const DhRow& row, double q) {
    const double theta = q + row.theta_offset;
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(row.alpha), sa = std::sin(row.alpha);

    Transform t;
    t.rotation(0, 0) = ct;  t.rotation(0, 1) = -st * ca; t.rotation(0, 2) = st * sa;
    t.rotation(1, 0) = st;  t.rotation(1, 1) = ct * ca;  t.rotation(1, 2) = -ct * sa;
    t.rotation(2, 0) = 0.0; t.rotation(2, 1) = sa;       t.rotation(2, 2) = ca;
    t.translation = {row.a * ct, row.a * st, row.d};
    return t;
}

std::array<Transform, kNumJoints> link_transforms(const ArmModel& model, const JointVector& q) {
    std::array<Transform, kNumJoints> out;
    Transform running = Transform::identity();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        running = running * dh_transform(model.dh[i], q[i]);
        out[i] = running;
    }
    return out;
}

Transform forward_kinematics(const ArmModel& model, const JointVector& q) {
    return link_transforms(model, q).back();
}

Vec3 wrist_centre(const ArmModel& model, const JointVector& q) {
    const Transform tool = forward_kinematics(model, q);
    return tool.translation - tool.axis(2) * model.dh[5].d;
}

Matrix<6, kNumJoints> jacobian(const ArmModel& model, const JointVector& q) {
    const auto frames = link_transforms(model, q);
    const Vec3 tool = frames.back().translation;

    Matrix<6, kNumJoints> j;
    Transform previous = Transform::identity();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
        // Every joint on this arm is revolute, so the column is the standard
        // (z x (p_tool - p_i), z) pair with z and p taken from the frame the
        // joint rotates in, which is frame i-1.
        const Vec3 z = previous.axis(2);
        const Vec3 p = previous.translation;
        const Vec3 linear = cross(z, tool - p);
        j(0, i) = linear.x; j(1, i) = linear.y; j(2, i) = linear.z;
        j(3, i) = z.x;      j(4, i) = z.y;      j(5, i) = z.z;
        previous = frames[i];
    }
    return j;
}

double manipulability(const Matrix<6, kNumJoints>& j) {
    // J J^T is 6x6, symmetric and positive semi-definite. Cyclic Jacobi
    // converges in a handful of sweeps at this size and needs no workspace.
    Matrix<6, 6> a = j * j.transposed();
    for (int sweep = 0; sweep < 30; ++sweep) {
        double off = 0.0;
        for (std::size_t p = 0; p < 6; ++p)
            for (std::size_t qq = p + 1; qq < 6; ++qq) off += a(p, qq) * a(p, qq);
        if (off < 1e-24) break;
        for (std::size_t p = 0; p < 6; ++p) {
            for (std::size_t qq = p + 1; qq < 6; ++qq) {
                if (std::fabs(a(p, qq)) < 1e-18) continue;
                const double theta = 0.5 * std::atan2(2.0 * a(p, qq), a(qq, qq) - a(p, p));
                const double c = std::cos(theta), s = std::sin(theta);
                for (std::size_t k = 0; k < 6; ++k) {
                    const double akp = a(k, p), akq = a(k, qq);
                    a(k, p) = c * akp - s * akq;
                    a(k, qq) = s * akp + c * akq;
                }
                for (std::size_t k = 0; k < 6; ++k) {
                    const double apk = a(p, k), aqk = a(qq, k);
                    a(p, k) = c * apk - s * aqk;
                    a(qq, k) = s * apk + c * aqk;
                }
            }
        }
    }
    double smallest = a(0, 0);
    for (std::size_t i = 1; i < 6; ++i) smallest = std::min(smallest, a(i, i));
    return std::sqrt(std::max(0.0, smallest));
}

}  // namespace ak
