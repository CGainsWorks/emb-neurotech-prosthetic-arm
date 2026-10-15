#include <random>

#include "arm_kinematics/transform.hpp"
#include "test_framework.hpp"

using namespace ak;

namespace {

constexpr double kPi = 3.14159265358979323846;

Mat3 random_rotation(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> angle(-kPi, kPi);
    return rotation_z(angle(rng)) * rotation_y(angle(rng)) * rotation_x(angle(rng));
}

double max_difference(const Mat3& a, const Mat3& b) {
    double worst = 0.0;
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            worst = std::max(worst, std::fabs(a(r, c) - b(r, c)));
    return worst;
}

}  // namespace

TEST(cross_product_obeys_the_right_hand_rule) {
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 y{0.0, 1.0, 0.0};
    const Vec3 z = cross(x, y);
    CHECK_NEAR(z.x, 0.0, 1e-15);
    CHECK_NEAR(z.y, 0.0, 1e-15);
    CHECK_NEAR(z.z, 1.0, 1e-15);
}

TEST(skew_matrix_reproduces_the_cross_product) {
    std::mt19937_64 rng(3);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);
    for (int trial = 0; trial < 100; ++trial) {
        const Vec3 a{dist(rng), dist(rng), dist(rng)};
        const Vec3 b{dist(rng), dist(rng), dist(rng)};
        const Vec3 direct = cross(a, b);
        const Vec3 via_matrix = skew(a) * b;
        CHECK_NEAR(direct.x, via_matrix.x, 1e-14);
        CHECK_NEAR(direct.y, via_matrix.y, 1e-14);
        CHECK_NEAR(direct.z, via_matrix.z, 1e-14);
    }
}

TEST(rotations_are_orthonormal) {
    std::mt19937_64 rng(5);
    for (int trial = 0; trial < 200; ++trial) {
        const Mat3 r = random_rotation(rng);
        const Mat3 should_be_identity = r * r.transposed();
        CHECK_NEAR(max_difference(should_be_identity, Mat3::identity()), 0.0, 1e-13);
    }
}

TEST(transform_inverse_undoes_the_transform) {
    std::mt19937_64 rng(9);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    for (int trial = 0; trial < 200; ++trial) {
        Transform t;
        t.rotation = random_rotation(rng);
        t.translation = {dist(rng), dist(rng), dist(rng)};

        const Transform round_trip = t * t.inverse();
        CHECK_NEAR(max_difference(round_trip.rotation, Mat3::identity()), 0.0, 1e-13);
        CHECK_NEAR(norm(round_trip.translation), 0.0, 1e-13);

        const Vec3 p{dist(rng), dist(rng), dist(rng)};
        const Vec3 back = t.inverse().apply(t.apply(p));
        CHECK_NEAR(norm(back - p), 0.0, 1e-13);
    }
}

TEST(rodrigues_and_rotation_error_are_inverses) {
    std::mt19937_64 rng(13);
    for (int trial = 0; trial < 500; ++trial) {
        const Mat3 from = random_rotation(rng);
        const Mat3 to = random_rotation(rng);
        const Vec3 w = rotation_error(from, to);
        const Mat3 reconstructed = rotation_from_vector(w) * from;
        CHECK_NEAR(max_difference(reconstructed, to), 0.0, 1e-11);
    }
}

TEST(rotation_error_is_zero_for_identical_rotations) {
    std::mt19937_64 rng(17);
    for (int trial = 0; trial < 100; ++trial) {
        const Mat3 r = random_rotation(rng);
        CHECK_NEAR(norm(rotation_error(r, r)), 0.0, 1e-12);
    }
}

TEST(rotation_error_handles_a_half_turn) {
    // The naive formula divides by sin(theta), which vanishes here. The wrist
    // reaches this, so it is not a corner case that can be waved away.
    for (int axis = 0; axis < 3; ++axis) {
        Mat3 half_turn;
        if (axis == 0) half_turn = rotation_x(kPi);
        if (axis == 1) half_turn = rotation_y(kPi);
        if (axis == 2) half_turn = rotation_z(kPi);

        const Vec3 w = rotation_error(Mat3::identity(), half_turn);
        CHECK_NEAR(norm(w), kPi, 1e-6);
        const Mat3 reconstructed = rotation_from_vector(w);
        CHECK_NEAR(max_difference(reconstructed, half_turn), 0.0, 1e-6);
    }
}

TEST(quaternion_conversion_stays_unit_length_near_a_half_turn) {
    // Shepperd's branch selection exists for exactly these rotations; the
    // trace-first formula loses half its digits here.
    std::mt19937_64 rng(19);
    std::uniform_real_distribution<double> nudge(-1e-7, 1e-7);
    for (int trial = 0; trial < 200; ++trial) {
        const Mat3 r = rotation_z(kPi + nudge(rng)) * rotation_x(nudge(rng));
        const Quaternion q = to_quaternion(r);
        const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        CHECK_NEAR(length, 1.0, 1e-12);
    }
}

TEST(quaternion_conversion_agrees_with_the_rotation_it_came_from) {
    std::mt19937_64 rng(23);
    for (int trial = 0; trial < 300; ++trial) {
        const Mat3 r = random_rotation(rng);
        const Quaternion q = to_quaternion(r);

        Mat3 rebuilt;
        rebuilt(0, 0) = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        rebuilt(0, 1) = 2.0 * (q.x * q.y - q.z * q.w);
        rebuilt(0, 2) = 2.0 * (q.x * q.z + q.y * q.w);
        rebuilt(1, 0) = 2.0 * (q.x * q.y + q.z * q.w);
        rebuilt(1, 1) = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
        rebuilt(1, 2) = 2.0 * (q.y * q.z - q.x * q.w);
        rebuilt(2, 0) = 2.0 * (q.x * q.z - q.y * q.w);
        rebuilt(2, 1) = 2.0 * (q.y * q.z + q.x * q.w);
        rebuilt(2, 2) = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);

        CHECK_NEAR(max_difference(rebuilt, r), 0.0, 1e-12);
    }
}

TEST(wrap_angle_lands_in_the_half_open_interval) {
    std::mt19937_64 rng(29);
    std::uniform_real_distribution<double> dist(-40.0, 40.0);
    for (int trial = 0; trial < 1000; ++trial) {
        const double a = dist(rng);
        const double wrapped = wrap_angle(a);
        CHECK(wrapped > -kPi - 1e-12);
        CHECK(wrapped <= kPi + 1e-12);
        // Same angle, different branch: the difference is a whole number of turns.
        const double turns = (a - wrapped) / (2.0 * kPi);
        CHECK_NEAR(turns, std::round(turns), 1e-9);
    }
}

TEST_MAIN()
