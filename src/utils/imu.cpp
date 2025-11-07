#include "srd/utils/imu.hpp"
#include "srd/utils/math.hpp"
#include "lgmath.hpp"

#include <opencv2/core/eigen.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

namespace srd::utils {

double preintegrate_gyro(const std::vector<Eigen::Vector4d>& imu_measurements,
                        double t_start_us, double t_end_us) {
    const double t_start_s = t_start_us * 1e-6;
    const double t_end_s   = t_end_us * 1e-6;

    if (t_end_s <= t_start_s) {
        throw std::invalid_argument("[preintegrate_gyro] t_end must be greater than t_start");
    }

    if (imu_measurements.empty()) {
        throw std::runtime_error("[preintegrate_gyro] No IMU measurements available");
    }

    // Find iterator range around t_start and t_end
    auto cmp = [](const Eigen::Vector4d& a, double b) { return a(0) < b; };
    auto it_start_next = std::lower_bound(imu_measurements.begin(), imu_measurements.end(), t_start_s, cmp);
    auto it_end_next   = std::lower_bound(imu_measurements.begin(), imu_measurements.end(), t_end_s, cmp);

    if (it_start_next == imu_measurements.begin() || it_end_next == imu_measurements.begin()) {
        std::cerr << "[Warning] t_start or t_end is before the first IMU measurement\n";
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (it_start_next == imu_measurements.end() || it_end_next == imu_measurements.end()) {
        std::cerr << "[Warning] t_start or t_end is beyond available IMU data\n";
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Integrate yaw rate over time
    Eigen::Matrix3d C_y = Eigen::Matrix3d::Identity();

    auto it = it_start_next - 1;
    double t_prev   = (*it)(0);
    double yaw_prev = (*it)(3);

    for (; it < it_end_next - 1; ++it) {
        double t_curr   = (*(it + 1))(0);
        double yaw_curr = (*(it + 1))(3);
        double dt       = t_curr - t_prev;

        C_y *= rpy_to_rot(0.0, 0.0, yaw_prev * dt);

        t_prev   = t_curr;
        yaw_prev = yaw_curr;
    }

    // Extract integrated yaw from final rotation
    return lgmath::so3::rot2vec(C_y).z();
}

Eigen::Vector4d get_imu_measurement(const std::vector<Eigen::Vector4d>& imu_measurements,
                                    double timestamp_us) {
    if (imu_measurements.empty()) {
        throw std::runtime_error("[get_imu_measurement] Empty IMU measurement list");
    }

    double t_s = timestamp_us * 1e-6;
    if (t_s < imu_measurements.front()(0) || t_s > imu_measurements.back()(0)) {
        std::cerr << "[Warning] Timestamp " << timestamp_us
                  << " out of IMU range (" << imu_measurements.front()(0)*1e6
                  << " to " << imu_measurements.back()(0)*1e6 << ")\n";
        return Eigen::Vector4d::Zero();
    }

    auto it = std::lower_bound(
        imu_measurements.begin(), imu_measurements.end(), t_s,
        [](const Eigen::Vector4d& a, double b) { return a(0) < b; });

    return *it;
}

std::vector<Eigen::Vector4d> load_all_imu_measurements(const std::filesystem::path& imu_meas_file) {
    std::ifstream imu_stream(imu_meas_file);
    if (!imu_stream.is_open()) {
        throw std::runtime_error("[load_all_imu_measurements] Failed to open file: " + imu_meas_file.string());
    }

    std::vector<Eigen::Vector4d> imu_measurements;
    imu_measurements.reserve(10000);  // optional preallocation

    std::string header;
    std::getline(imu_stream, header);  // skip header

    for (std::string line; std::getline(imu_stream, line);) {
        std::stringstream ss(line);
        std::vector<double> values;
        for (std::string str; std::getline(ss, str, ',');)
            values.push_back(std::stod(str));

        Eigen::Vector4d meas;
        auto filename = imu_meas_file.filename().string();

        if (filename == "imu.csv" || filename == "imu_raw.csv") {
            meas << values[0], values[3], values[2], values[1]; // timestamp, angvel_x, angvel_y, angvel_z
        } else if (filename == "dmu_imu.csv") {
            meas << values[0] / 1e9, values[7], values[8], values[9];
        } else {
            std::cerr << "[Warning] Unknown IMU file format: " << filename << '\n';
            continue;
        }

        imu_measurements.push_back(meas);
    }

    return imu_measurements;
}

}  // namespace srd::utils
