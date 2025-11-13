#include "srd/odometry/odometry_config.hpp"

namespace srd::odometry {

DopplerOdometry::Options load_odometry_options(const YAML::Node& config) {
  DopplerOdometry::Options opts;

  auto filter = config["filter"];
  if (filter) {
    if (filter["v_0"]) {
      auto v_0 = filter["v_0"].as<std::vector<double>>();
      opts.v_0 = Eigen::Vector2d(v_0[0], v_0[1]);
    }
    if (filter["v_0_std"]) {
      auto v_0_std = filter["v_0_std"].as<std::vector<double>>();
      opts.v_0_std = Eigen::Vector2d(v_0_std[0], v_0_std[1]);
    }
    if (filter["wnoa_std"]) {
      auto wnoa_std = filter["wnoa_std"].as<std::vector<double>>();
      opts.wnoa_std = Eigen::Vector2d(wnoa_std[0], wnoa_std[1]);
    }
    if (filter["imu_file_name"]) opts.imu_file_name = filter["imu_file_name"].as<std::string>();
  }

  return opts;
}

} // namespace srd::odometry
