#pragma once
#include <memory>
#include <Eigen/Dense>
#include <vector>

#include "srd/common/common.hpp"
#include "srd/odometry/trajectory.hpp"

namespace srd::odometry {

class DopplerOdometry {
 public:
  struct Options {
    Eigen::Vector2d v_0{0.0, 0.0};
    Eigen::Vector2d v_0_std{0.1, 0.1};
    Eigen::Vector2d wnoa_std{0.1, 0.1};
    std::string imu_file_name;
  };

  DopplerOdometry();
  explicit DopplerOdometry(const Options& options);

  void progress_odometry(const DopplerScan& scan, int64_t timestamp);

 private:
  Options options_;
  Trajectory trajectory_;
};

} // namespace srd::odometry
