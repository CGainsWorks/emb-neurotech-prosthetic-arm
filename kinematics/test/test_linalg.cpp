#include <random>

#include "arm_kinematics/linalg.hpp"
#include "test_framework.hpp"

using namespace ak;

TEST(identity_is_multiplicative_identity) {
    Matrix<3, 3> a;
    a(0, 0) = 1.0; a(0, 1) = 2.0; a(0, 2) = 3.0;
    a(1, 0) = 4.0; a(1, 1) = 5.0; a(1, 2) = 6.0;
    a(2, 0) = 7.0; a(2, 1) = 8.0; a(2, 2) = 10.0;

    const Matrix<3, 3> product = a * Matrix<3, 3>::identity();
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c) CHECK_NEAR(product(r, c), a(r, c), 1e-15);
}

TEST(transpose_is_an_involution) {
    Matrix<2, 3> a;
    double v = 0.0;
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 3; ++c) a(r, c) = (v += 1.5);

    const Matrix<2, 3> back = a.transposed().transposed();
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 3; ++c) CHECK_NEAR(back(r, c), a(r, c), 1e-15);
}

TEST(solve_recovers_a_known_solution) {
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);

    for (int trial = 0; trial < 200; ++trial) {
        Matrix<6, 6> a;
        for (std::size_t r = 0; r < 6; ++r)
            for (std::size_t c = 0; c < 6; ++c) a(r, c) = dist(rng);
        // Diagonal dominance keeps the random matrix invertible without having
        // to reject-sample.
        for (std::size_t i = 0; i < 6; ++i) a(i, i) += 8.0;

        Vector<6> x_true;
        for (std::size_t i = 0; i < 6; ++i) x_true(i, 0) = dist(rng);
        const Vector<6> b = a * x_true;

        Vector<6> x;
        CHECK(solve<6, 1>(a, b, x));
        for (std::size_t i = 0; i < 6; ++i) CHECK_NEAR(x(i, 0), x_true(i, 0), 1e-9);
    }
}

TEST(solve_rejects_a_singular_matrix) {
    Matrix<3, 3> a;
    a(0, 0) = 1.0; a(0, 1) = 2.0; a(0, 2) = 3.0;
    a(1, 0) = 2.0; a(1, 1) = 4.0; a(1, 2) = 6.0;  // twice row 0
    a(2, 0) = 1.0; a(2, 1) = 1.0; a(2, 2) = 1.0;

    Vector<3> b, x;
    b(0, 0) = 1.0;
    CHECK(!solve<3, 1>(a, b, x));
}

TEST(solve_needs_pivoting_for_a_zero_leading_entry) {
    // Without partial pivoting this divides by zero on the first step. The
    // Jacobian hits exactly this shape at a shoulder singularity.
    Matrix<3, 3> a;
    a(0, 0) = 0.0; a(0, 1) = 2.0; a(0, 2) = 1.0;
    a(1, 0) = 1.0; a(1, 1) = 0.0; a(1, 2) = 3.0;
    a(2, 0) = 4.0; a(2, 1) = 1.0; a(2, 2) = 0.0;

    Vector<3> x_true;
    x_true(0, 0) = 1.0; x_true(1, 0) = -2.0; x_true(2, 0) = 0.5;
    const Vector<3> b = a * x_true;

    Vector<3> x;
    CHECK(solve<3, 1>(a, b, x));
    for (std::size_t i = 0; i < 3; ++i) CHECK_NEAR(x(i, 0), x_true(i, 0), 1e-12);
}

TEST(damped_least_squares_matches_the_exact_inverse_when_damping_is_tiny) {
    std::mt19937_64 rng(11);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Matrix<6, 6> j;
    for (std::size_t r = 0; r < 6; ++r)
        for (std::size_t c = 0; c < 6; ++c) j(r, c) = dist(rng);
    for (std::size_t i = 0; i < 6; ++i) j(i, i) += 5.0;

    Vector<6> e;
    for (std::size_t i = 0; i < 6; ++i) e(i, 0) = dist(rng);

    Vector<6> damped;
    CHECK(damped_least_squares(j, e, 1e-9, damped));

    Vector<6> exact;
    CHECK(solve<6, 1>(j, e, exact));
    for (std::size_t i = 0; i < 6; ++i) CHECK_NEAR(damped(i, 0), exact(i, 0), 1e-6);
}

TEST(damped_least_squares_still_answers_at_a_singularity) {
    // A Jacobian of rank one. The undamped inverse does not exist; the damped
    // one must still return a finite step in the right direction.
    Matrix<6, 6> j;
    for (std::size_t r = 0; r < 6; ++r) j(r, 0) = 1.0;

    Vector<6> e;
    for (std::size_t i = 0; i < 6; ++i) e(i, 0) = 0.5;

    Vector<6> step;
    CHECK(damped_least_squares(j, e, 1e-2, step));
    for (std::size_t i = 0; i < 6; ++i) CHECK(std::isfinite(step(i, 0)));
    CHECK(step(0, 0) > 0.0);
    for (std::size_t i = 1; i < 6; ++i) CHECK_NEAR(step(i, 0), 0.0, 1e-15);
}

TEST_MAIN()
