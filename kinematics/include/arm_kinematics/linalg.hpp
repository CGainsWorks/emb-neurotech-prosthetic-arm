// Fixed-size dense linear algebra, sized at compile time.
//
// Deliberately dependency-free. The same headers are compiled into the
// firmware, where Eigen's dynamic allocation and template depth are both
// unwelcome and into the ROS 2 nodes, where an extra dependency buys nothing
// at these matrix sizes. Nothing here is larger than 6x6, so the naive
// algorithms below are the right ones: an LU factorisation of a 6x6 costs
// about 100 flops and any cleverness would be measuring itself.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace ak {

// A row-major R x C matrix of doubles.
template <std::size_t R, std::size_t C>
struct Matrix {
    std::array<double, R * C> data{};

    static constexpr std::size_t rows = R;
    static constexpr std::size_t cols = C;

    constexpr double& operator()(std::size_t r, std::size_t c) { return data[r * C + c]; }
    constexpr double operator()(std::size_t r, std::size_t c) const { return data[r * C + c]; }

    static Matrix zero() { return Matrix{}; }

    static Matrix identity() {
        Matrix m;
        for (std::size_t i = 0; i < std::min(R, C); ++i) m(i, i) = 1.0;
        return m;
    }

    Matrix<C, R> transposed() const {
        Matrix<C, R> t;
        for (std::size_t r = 0; r < R; ++r)
            for (std::size_t c = 0; c < C; ++c) t(c, r) = (*this)(r, c);
        return t;
    }

    double max_abs() const {
        double m = 0.0;
        for (double v : data) m = std::max(m, std::fabs(v));
        return m;
    }
};

template <std::size_t R, std::size_t C>
Matrix<R, C> operator+(const Matrix<R, C>& a, const Matrix<R, C>& b) {
    Matrix<R, C> out;
    for (std::size_t i = 0; i < R * C; ++i) out.data[i] = a.data[i] + b.data[i];
    return out;
}

template <std::size_t R, std::size_t C>
Matrix<R, C> operator-(const Matrix<R, C>& a, const Matrix<R, C>& b) {
    Matrix<R, C> out;
    for (std::size_t i = 0; i < R * C; ++i) out.data[i] = a.data[i] - b.data[i];
    return out;
}

template <std::size_t R, std::size_t C>
Matrix<R, C> operator*(const Matrix<R, C>& a, double s) {
    Matrix<R, C> out;
    for (std::size_t i = 0; i < R * C; ++i) out.data[i] = a.data[i] * s;
    return out;
}

template <std::size_t R, std::size_t K, std::size_t C>
Matrix<R, C> operator*(const Matrix<R, K>& a, const Matrix<K, C>& b) {
    Matrix<R, C> out;
    for (std::size_t r = 0; r < R; ++r)
        for (std::size_t k = 0; k < K; ++k) {
            const double av = a(r, k);
            if (av == 0.0) continue;
            for (std::size_t c = 0; c < C; ++c) out(r, c) += av * b(k, c);
        }
    return out;
}

// A column vector is a matrix with one column, so the operators above already
// cover it; this alias just makes the intent readable at call sites.
template <std::size_t N>
using Vector = Matrix<N, 1>;

template <std::size_t N>
Vector<N> to_vector(const std::array<double, N>& a) {
    Vector<N> v;
    for (std::size_t i = 0; i < N; ++i) v(i, 0) = a[i];
    return v;
}

template <std::size_t N>
std::array<double, N> to_array(const Vector<N>& v) {
    std::array<double, N> a{};
    for (std::size_t i = 0; i < N; ++i) a[i] = v(i, 0);
    return a;
}

template <std::size_t N>
double dot(const Vector<N>& a, const Vector<N>& b) {
    double s = 0.0;
    for (std::size_t i = 0; i < N; ++i) s += a(i, 0) * b(i, 0);
    return s;
}

template <std::size_t N>
double norm(const Vector<N>& v) {
    return std::sqrt(dot(v, v));
}

// Solve A x = b by LU with partial pivoting. Returns false when A is singular
// to working precision, in which case x is left untouched. Partial pivoting is
// not optional here: the manipulator Jacobian is exactly singular at the
// shoulder and elbow singularities and those are reachable configurations,
// not pathological inputs.
template <std::size_t N, std::size_t C>
bool solve(Matrix<N, N> a, Matrix<N, C> b, Matrix<N, C>& x) {
    std::array<std::size_t, N> piv{};
    for (std::size_t i = 0; i < N; ++i) piv[i] = i;

    for (std::size_t k = 0; k < N; ++k) {
        std::size_t best = k;
        double best_mag = std::fabs(a(k, k));
        for (std::size_t r = k + 1; r < N; ++r) {
            const double mag = std::fabs(a(r, k));
            if (mag > best_mag) {
                best_mag = mag;
                best = r;
            }
        }
        if (best_mag < 1e-14) return false;
        if (best != k) {
            for (std::size_t c = 0; c < N; ++c) std::swap(a(k, c), a(best, c));
            for (std::size_t c = 0; c < C; ++c) std::swap(b(k, c), b(best, c));
            std::swap(piv[k], piv[best]);
        }
        for (std::size_t r = k + 1; r < N; ++r) {
            const double factor = a(r, k) / a(k, k);
            if (factor == 0.0) continue;
            a(r, k) = 0.0;
            for (std::size_t c = k + 1; c < N; ++c) a(r, c) -= factor * a(k, c);
            for (std::size_t c = 0; c < C; ++c) b(r, c) -= factor * b(k, c);
        }
    }

    for (std::size_t c = 0; c < C; ++c) {
        for (std::size_t ri = N; ri-- > 0;) {
            double s = b(ri, c);
            for (std::size_t k = ri + 1; k < N; ++k) s -= a(ri, k) * x(k, c);
            x(ri, c) = s / a(ri, ri);
        }
    }
    return true;
}

// Damped least squares inverse applied to a right-hand side:
//   dq = J^T (J J^T + lambda^2 I)^-1 e
//
// Written this way rather than as an explicit pseudo-inverse because the
// damped normal equations are always solvable for lambda > 0, including at a
// singularity, which is the entire reason for using damping.
template <std::size_t R, std::size_t C>
bool damped_least_squares(const Matrix<R, C>& j, const Vector<R>& e, double lambda,
                          Vector<C>& out) {
    Matrix<R, R> jjt = j * j.transposed();
    for (std::size_t i = 0; i < R; ++i) jjt(i, i) += lambda * lambda;
    Vector<R> y;
    if (!solve<R, 1>(jjt, e, y)) return false;
    out = j.transposed() * y;
    return true;
}

}  // namespace ak
