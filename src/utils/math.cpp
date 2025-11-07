#include "srd/utils/math.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <initializer_list>

namespace srd::utils {

Eigen::Matrix3d to_roll(double r) {
    Eigen::Matrix3d roll;
    roll << 1, 0, 0, 0, cos(r), sin(r), 0, -sin(r), cos(r);
    return roll;
}

Eigen::Matrix3d to_pitch(double p) {
    Eigen::Matrix3d pitch;
    pitch << cos(p), 0, -sin(p), 0, 1, 0, sin(p), 0, cos(p);
    return pitch;
}

Eigen::Matrix3d to_yaw(double y) {
    Eigen::Matrix3d yaw;
    yaw << cos(y), sin(y), 0, -sin(y), cos(y), 0, 0, 0, 1;
    return yaw;
}

Eigen::Matrix3d rpy_to_rot(double r, double p, double y) {
    return to_roll(r) * to_pitch(p) * to_yaw(y);
}

double quadratic_interpolation(int64_t t, int64_t t0, double v0, int64_t t1, double v1, int64_t t2, double v2) {
    if (t0 == t1 || t0 == t2 || t1 == t2) {
        throw std::runtime_error("[quadratic_interpolation] Timestamps for quadratic interpolation must be distinct");
    }
    if (t < std::min({t0, t1, t2}) || t > std::max({t0, t1, t2})) {
        throw std::runtime_error("[quadratic_interpolation] t is out of bounds for quadratic interpolation");
    }
    if (!(t0 < t1 && t1 < t2)) {
        throw std::runtime_error("[quadratic_interpolation] Timestamps must be in ascending order for quadratic interpolation");
    }

    double td = static_cast<double>(t);
    double td0 = static_cast<double>(t0);
    double td1 = static_cast<double>(t1);
    double td2 = static_cast<double>(t2);

    double L0 = ((td - td1) * (td - td2)) / ((td0 - td1) * (td0 - td2));
    double L1 = ((td - td0) * (td - td2)) / ((td1 - td0) * (td1 - td2));
    double L2 = ((td - td0) * (td - td1)) / ((td2 - td0) * (td2 - td1));

    return v0 * L0 + v1 * L1 + v2 * L2;
}

double linear_interpolation(int64_t t, int64_t t0, double v0, int64_t t1, double v1) {
    if (t1 == t0) {
        throw std::runtime_error("[linear_interpolation] Timestamps for linear interpolation must be distinct");
    }
    if (t < std::min(t0, t1) || t > std::max(t0, t1)) {
        throw std::runtime_error("[linear_interpolation] t is out of bounds for linear interpolation");
    }
    if (!(t0 < t1)) {
        throw std::runtime_error("[linear_interpolation] Timestamps must be in ascending order for linear interpolation");
    }

    double alpha = static_cast<double>(t - t0) / static_cast<double>(t1 - t0);
    return (1 - alpha) * v0 + alpha * v1;
}

} // namespace srd::utils