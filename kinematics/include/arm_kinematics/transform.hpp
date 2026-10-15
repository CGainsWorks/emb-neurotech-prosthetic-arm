// Rigid-body transforms and the small amount of rotation algebra the arm needs.
#pragma once

#include <array>
#include <cmath>

#include "arm_kinematics/linalg.hpp"

namespace ak {

struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};

    double operator[](std::size_t i) const { return i == 0 ? x : (i == 1 ? y : z); }
    double& operator[](std::size_t i) { return i == 0 ? x : (i == 1 ? y : z); }
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(double s, const Vec3& a) { return a * s; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

using Mat3 = Matrix<3, 3>;

inline Vec3 operator*(const Mat3& m, const Vec3& v) {
    return {m(0, 0) * v.x + m(0, 1) * v.y + m(0, 2) * v.z,
            m(1, 0) * v.x + m(1, 1) * v.y + m(1, 2) * v.z,
            m(2, 0) * v.x + m(2, 1) * v.y + m(2, 2) * v.z};
}

// Skew-symmetric matrix such that skew(a) * b == cross(a, b).
inline Mat3 skew(const Vec3& a) {
    Mat3 m;
    m(0, 1) = -a.z; m(0, 2) = a.y;
    m(1, 0) = a.z;  m(1, 2) = -a.x;
    m(2, 0) = -a.y; m(2, 1) = a.x;
    return m;
}

// A homogeneous transform, stored as rotation plus translation rather than a
// 4x4, because every operation the arm performs treats the two separately.
struct Transform {
    Mat3 rotation{Mat3::identity()};
    Vec3 translation{};

    static Transform identity() { return Transform{}; }

    Transform operator*(const Transform& rhs) const {
        Transform out;
        out.rotation = rotation * rhs.rotation;
        out.translation = translation + rotation * rhs.translation;
        return out;
    }

    Vec3 apply(const Vec3& p) const { return translation + rotation * p; }

    Transform inverse() const {
        Transform out;
        out.rotation = rotation.transposed();
        out.translation = out.rotation * translation * -1.0;
        return out;
    }

    // Column i of the rotation matrix: the arm's x, y and z axes in world.
    Vec3 axis(std::size_t i) const { return {rotation(0, i), rotation(1, i), rotation(2, i)}; }
};

struct Quaternion {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

// Shepperd's method: pick the largest diagonal term so the square root never
// divides by something near zero. The naive w-first formula loses most of its
// precision for rotations near 180 degrees, which the wrist reaches.
inline Quaternion to_quaternion(const Mat3& r) {
    const double trace = r(0, 0) + r(1, 1) + r(2, 2);
    Quaternion q;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (r(2, 1) - r(1, 2)) / s;
        q.y = (r(0, 2) - r(2, 0)) / s;
        q.z = (r(1, 0) - r(0, 1)) / s;
    } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
        const double s = std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
        q.w = (r(2, 1) - r(1, 2)) / s;
        q.x = 0.25 * s;
        q.y = (r(0, 1) + r(1, 0)) / s;
        q.z = (r(0, 2) + r(2, 0)) / s;
    } else if (r(1, 1) > r(2, 2)) {
        const double s = std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
        q.w = (r(0, 2) - r(2, 0)) / s;
        q.x = (r(0, 1) + r(1, 0)) / s;
        q.y = 0.25 * s;
        q.z = (r(1, 2) + r(2, 1)) / s;
    } else {
        const double s = std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
        q.w = (r(1, 0) - r(0, 1)) / s;
        q.x = (r(0, 2) + r(2, 0)) / s;
        q.y = (r(1, 2) + r(2, 1)) / s;
        q.z = 0.25 * s;
    }
    return q;
}

inline Mat3 rotation_x(double a) {
    Mat3 m = Mat3::identity();
    m(1, 1) = std::cos(a); m(1, 2) = -std::sin(a);
    m(2, 1) = std::sin(a); m(2, 2) = std::cos(a);
    return m;
}

inline Mat3 rotation_y(double a) {
    Mat3 m = Mat3::identity();
    m(0, 0) = std::cos(a);  m(0, 2) = std::sin(a);
    m(2, 0) = -std::sin(a); m(2, 2) = std::cos(a);
    return m;
}

inline Mat3 rotation_z(double a) {
    Mat3 m = Mat3::identity();
    m(0, 0) = std::cos(a); m(0, 1) = -std::sin(a);
    m(1, 0) = std::sin(a); m(1, 1) = std::cos(a);
    return m;
}

// The rotation vector (axis times angle) taking `from` to `to`. This is the
// orientation error the numerical solver and the Cartesian controller drive to
// zero; expressing it as a rotation vector rather than three Euler angles is
// what keeps the error linear in the Jacobian near the solution.
inline Vec3 rotation_error(const Mat3& from, const Mat3& to) {
    const Mat3 r = to * from.transposed();
    const double trace = r(0, 0) + r(1, 1) + r(2, 2);
    double cos_theta = (trace - 1.0) * 0.5;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    const double theta = std::acos(cos_theta);
    const Vec3 axis_sin{(r(2, 1) - r(1, 2)) * 0.5, (r(0, 2) - r(2, 0)) * 0.5,
                        (r(1, 0) - r(0, 1)) * 0.5};
    const double sin_theta = std::sin(theta);
    // Near zero the vector part is already the answer to first order; near pi
    // sin(theta) vanishes and the vector part carries no direction, so fall
    // back to the symmetric part of R.
    if (theta < 1e-8) return axis_sin;
    if (sin_theta < 1e-8) {
        Vec3 axis{std::sqrt(std::max(0.0, (r(0, 0) + 1.0) * 0.5)),
                  std::sqrt(std::max(0.0, (r(1, 1) + 1.0) * 0.5)),
                  std::sqrt(std::max(0.0, (r(2, 2) + 1.0) * 0.5))};
        if (axis_sin.x < 0.0) axis.x = -axis.x;
        if (axis_sin.y < 0.0) axis.y = -axis.y;
        if (axis_sin.z < 0.0) axis.z = -axis.z;
        const double n = norm(axis);
        return n > 0.0 ? axis * (theta / n) : Vec3{};
    }
    return axis_sin * (theta / sin_theta);
}

// Rodrigues: the rotation of |v| radians about v. The inverse of
// rotation_error and what pose interpolation needs to walk along a rotation.
inline Mat3 rotation_from_vector(const Vec3& v) {
    const double angle = norm(v);
    if (angle < 1e-12) return Mat3::identity();
    const Vec3 axis = v * (1.0 / angle);
    const Mat3 k = skew(axis);
    return Mat3::identity() + k * std::sin(angle) + (k * k) * (1.0 - std::cos(angle));
}

// Wrap to (-pi, pi]. Joint commands are compared and interpolated as angles,
// so every path through the solver has to agree on a branch.
inline double wrap_angle(double a) {
    constexpr double kTwoPi = 6.283185307179586;
    a = std::fmod(a + 3.141592653589793, kTwoPi);
    if (a <= 0.0) a += kTwoPi;
    return a - 3.141592653589793;
}

}  // namespace ak
