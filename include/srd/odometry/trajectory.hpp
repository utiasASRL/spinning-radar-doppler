#pragma once
#include <vector>
#include <Eigen/Dense>

namespace srd::odometry {

using ArrayMatrix4d = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;
using ArrayPoses = ArrayMatrix4d;

struct TrajectoryFrame {
  TrajectoryFrame() = default;

  int64_t begin_timestamp = 0;  // micro seconds
  int64_t end_timestamp = 2;    // micro seconds
  int64_t query_timestamp = 1;  // micro seconds, the time at which to query the pose/vel evaluator for gt

  Eigen::Matrix<double, 6, 1> varpi;  // estimate at end_timestamp

};

using Trajectory = std::vector<TrajectoryFrame>;

} // namespace srd::odometry