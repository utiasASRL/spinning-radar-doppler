#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <vector>
#include <cstdint>

namespace srd::utils {

// Preintegrate IMU measurements between two timestamps
// A NaN is returned if no gyro measurements are available in the range
double preintegrate_gyro(const std::vector<Eigen::Vector4d>& imu_measurements,
                        double t_start_us, double t_end_us);

// Retrieve the IMU measurement closest to a given timestamp (microseconds)
Eigen::Vector4d get_imu_measurement(const std::vector<Eigen::Vector4d>& imu_measurements,
                                    double timestamp_us);

// Load all IMU measurements from a CSV file (imu.csv or dmu_imu.csv)
std::vector<Eigen::Vector4d> load_all_imu_measurements(const std::filesystem::path& imu_meas_file);

}  // namespace srd::utils
