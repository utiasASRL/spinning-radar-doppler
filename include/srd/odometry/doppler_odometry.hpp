# pragma once

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "srd/common/common.hpp"
#include "srd/extractor/doppler_extractor.hpp"
#include <optional>

namespace srd::odometry {

class DopplerOdometry {
 public:
  using Ptr = std::shared_ptr<DopplerOdometry>;
  using ConstPtr = std::shared_ptr<const DopplerOdometry>;

  struct Options {
    // Initial condition for the vehicle velocity [vx, vy] (m/s)
    Eigen::Vector2d v_0 = Eigen::Vector2d(0.0, 0.0);
    // Standard deviation for the initial condition on velocity (m/s)
    Eigen::Vector2d v_0_std = Eigen::Vector2d(0.1, 0.1);
    // Standard deviation for the white noise on acceleration assumption (m/s^2)
    Eigen::Vector2d wnoa_std = Eigen::Vector2d(0.1, 0.1);

    // IMU file name
    std::string imu_file_name = "";
  };

  DopplerOdometry();
  explicit DopplerOdometry(const Options& options);

  void compute_odometry(const std::vector<DopplerScan>& doppler_scans,
                        std::vector<Eigen::Matrix4d>& poses) const;

 private:
  Options options_;
};

} // namespace srd::odometry