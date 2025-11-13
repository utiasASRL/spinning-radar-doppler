#pragma once

#include <yaml-cpp/yaml.h>
#include "srd/odometry/doppler_odometry.hpp"

namespace srd::odometry {

DopplerOdometry::Options load_odometry_options(const YAML::Node& config);

} // namespace srd::odometry
