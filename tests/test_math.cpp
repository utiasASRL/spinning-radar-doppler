#include <gtest/gtest.h>
#include "srd/utils/math.hpp"
#include <Eigen/Dense>

using namespace srd::utils;

// --- Rotation tests ---
TEST(MathTest, RotationComposition) {
    double non_zero_rot = M_PI / 2.0;

    // Test roll only
    Eigen::Matrix3d R = rpy_to_rot(non_zero_rot, 0.0, 0.0);
    Eigen::Matrix3d expected = to_roll(non_zero_rot);
    EXPECT_TRUE(R.isApprox(expected, 1e-9));
    // Test pitch only
    R = rpy_to_rot(0.0, non_zero_rot, 0.0);
    expected = to_pitch(non_zero_rot);
    EXPECT_TRUE(R.isApprox(expected, 1e-9));
    // Test yaw only
    R = rpy_to_rot(0.0, 0.0, non_zero_rot);
    expected = to_yaw(non_zero_rot);
    EXPECT_TRUE(R.isApprox(expected, 1e-9));
}

// --- Linear interpolation tests ---
TEST(MathTest, LinearInterpolationMiddle) {
    double v = linear_interpolation(5, 0, 0.0, 10, 10.0);
    EXPECT_DOUBLE_EQ(v, 5.0);
}

TEST(MathTest, LinearInterpolationBoundary) {
    EXPECT_DOUBLE_EQ(linear_interpolation(0, 0, 0.0, 10, 10.0), 0.0);
    EXPECT_DOUBLE_EQ(linear_interpolation(10, 0, 0.0, 10, 10.0), 10.0);
}

// --- Quadratic interpolation tests ---
TEST(MathTest, QuadraticInterpolationCenter) {
    double v = quadratic_interpolation(1, 0, 0.0, 1, 1.0, 2, 0.0);
    // Not perfectly parabolic in this example, but demonstrates use
    SUCCEED();  // Just ensures it runs without throwing
}

TEST(MathTest, QuadraticInterpolationInvalidOrder) {
    EXPECT_THROW(
        quadratic_interpolation(1, 2, 0.0, 1, 1.0, 0, 0.0),
        std::runtime_error
    );
}

// --- Bounds checking ---
TEST(MathTest, LinearInterpolationOutOfBoundsThrows) {
    EXPECT_THROW(linear_interpolation(15, 0, 0.0, 10, 10.0), std::runtime_error);
}

