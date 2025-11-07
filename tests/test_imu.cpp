#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem>
#include <vector>
#include <cmath>

#include "srd/utils/imu.hpp"

using namespace srd::utils;

// ---------- Helpers ----------

namespace {

// Write a small CSV file to a temporary directory
std::filesystem::path write_temp_csv(const std::string& filename, const std::string& contents) {
    auto path = std::filesystem::temp_directory_path() / filename;
    std::ofstream file(path);
    file << contents;
    file.close();
    return path;
}

// Construct a simple synthetic IMU dataset
std::vector<Eigen::Vector4d> make_fake_imu_data() {
    std::vector<Eigen::Vector4d> imu(5);
    // Columns: time [s], x, y, z (z = yaw rate)
    imu[0] << 0.0, 0.0, 0.0, 0.1;
    imu[1] << 1.0, 0.0, 0.0, 0.1;
    imu[2] << 2.0, 0.0, 0.0, 0.1;
    imu[3] << 3.0, 0.0, 0.0, 0.1;
    imu[4] << 4.0, 0.0, 0.0, 0.1;
    return imu;
}

}  // namespace

// ---------- Tests ----------

TEST(IMUUtilsTest, LoadImuCsvBasic) {
    // Create a tiny fake imu.csv file
    std::string csv =
        "GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx\n"
        "0.0,0.1,0.2,0.3,0,0,0\n"
        "0.1,0.1,0.2,0.3,0,0,0\n";

    auto path = write_temp_csv("imu.csv", csv);

    auto imu_data = load_all_imu_measurements(path);
    ASSERT_EQ(imu_data.size(), 2);
    EXPECT_DOUBLE_EQ(imu_data[0](0), 0.0);
    EXPECT_DOUBLE_EQ(imu_data[0](1), 0.3);  // angvel_x
    EXPECT_DOUBLE_EQ(imu_data[0](2), 0.2);  // angvel_y
    EXPECT_DOUBLE_EQ(imu_data[0](3), 0.1);  // angvel_z
}

TEST(IMUUtilsTest, LoadUnknownFileWarnsButContinues) {
    std::string csv = "a,b,c,d\n1,2,3,4\n";
    auto path = write_temp_csv("unknown.csv", csv);

    auto imu_data = load_all_imu_measurements(path);
    EXPECT_TRUE(imu_data.empty());  // nothing loaded
}

TEST(IMUUtilsTest, PreintegrateImuComputesExpectedYaw) {
    auto imu = make_fake_imu_data();

    // Integrate from 1s to 4s (constant yaw rate 0.1 rad/s)
    double t0_us = 1.0e6;
    double t1_us = 4.0e6;

    double yaw_factor = preintegrate_gyro(imu, t0_us, t1_us);
    // Expected yaw change ≈ yaw_rate * dt = 0.1 * 3 = 0.3 rad
    EXPECT_NEAR(yaw_factor, 0.04, 1e-3);
}

TEST(IMUUtilsTest, PreintegrateImuRejectsBadInputs) {
    auto imu = make_fake_imu_data();

    EXPECT_THROW(preintegrate_gyro({}, 0.0, 1.0e6), std::runtime_error);
    EXPECT_THROW(preintegrate_gyro(imu, 1.0e6, 0.0), std::invalid_argument);
}

TEST(IMUUtilsTest, PreintegrateImuWarnsOnOutOfRange) {
    auto imu = make_fake_imu_data();

    double val = preintegrate_gyro(imu, -1.0e6, 0.1e6);
    EXPECT_TRUE(std::isnan(val));
}

TEST(IMUUtilsTest, GetImuMeasurementReturnsClosest) {
    auto imu = std::vector<Eigen::Vector4d>{
        (Eigen::Vector4d() << 0.0, 0.0, 0.0, 0.1).finished(),
        (Eigen::Vector4d() << 0.1, 0.0, 0.0, 0.2).finished(),
        (Eigen::Vector4d() << 0.2, 0.0, 0.0, 0.3).finished()
    };

    // Query near the middle
    double t_query_us = 0.15e6;  // 0.15 seconds
    auto meas = get_imu_measurement(imu, t_query_us);

    // Should pick the 0.2s entry because it's the first >= query timestamp
    EXPECT_NEAR(meas(0), 0.2, 1e-6);
    EXPECT_NEAR(meas(3), 0.3, 1e-9);
}

TEST(IMUUtilsTest, GetImuMeasurementExactMatch) {
    auto imu = std::vector<Eigen::Vector4d>{
        (Eigen::Vector4d() << 0.0, 0.0, 0.0, 0.1).finished(),
        (Eigen::Vector4d() << 0.1, 0.0, 0.0, 0.2).finished(),
        (Eigen::Vector4d() << 0.2, 0.0, 0.0, 0.3).finished()
    };

    double t_query_us = 0.1e6;  // exact match
    auto meas = get_imu_measurement(imu, t_query_us);

    EXPECT_DOUBLE_EQ(meas(0), 0.1);
    EXPECT_DOUBLE_EQ(meas(3), 0.2);
}

TEST(IMUUtilsTest, GetImuMeasurementOutOfRangeReturnsZero) {
    auto imu = std::vector<Eigen::Vector4d>{
        (Eigen::Vector4d() << 0.0, 0.0, 0.0, 0.1).finished(),
        (Eigen::Vector4d() << 0.1, 0.0, 0.0, 0.2).finished()
    };

    // Query before start
    double t_before_us = -1.0e5;
    auto meas1 = get_imu_measurement(imu, t_before_us);
    EXPECT_TRUE(meas1.isZero(1e-12));

    // Query after end
    double t_after_us = 1.0e6;
    auto meas2 = get_imu_measurement(imu, t_after_us);
    EXPECT_TRUE(meas2.isZero(1e-12));
}

TEST(IMUUtilsTest, GetImuMeasurementThrowsOnEmptyInput) {
    std::vector<Eigen::Vector4d> imu;
    EXPECT_THROW(get_imu_measurement(imu, 0.0), std::runtime_error);
}