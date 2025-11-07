#pragma once

#include <Eigen/Dense>

namespace srd::utils {

// Convert roll angles to rotation matrix
// This uses the robotics convention, which may yield a transpose of other conventions
Eigen::Matrix3d to_roll(double r);
Eigen::Matrix3d to_pitch(double p);
Eigen::Matrix3d to_yaw(double y);
Eigen::Matrix3d rpy_to_rot(double r, double p, double y);

// Quadratic interpolation function between three points (t0, v0), (t1, v1), (t2, v2)
// Intended to be used for microsecond time, velocity pairs, but is general in nature
double quadratic_interpolation(int64_t t, int64_t t0, double v0, int64_t t1, double v1, int64_t t2, double v2);

// Linear interpolation function between two points (t0, v0), (t1, v1)
// Intended to be used for microsecond time, velocity pairs, but is general in nature
double linear_interpolation(int64_t t, int64_t t0, double v0, int64_t t1, double v1);

} // namespace srd::utils