#pragma once
#include <vector>
#include <Eigen/Dense>
#include <cstdint>

namespace srd::common {
// Represents one radar azimuth measurement
struct DopplerAzimuth {
  double radial_velocity = 0.0;  // Radial velocity of the point
  int64_t timestamp = 0;         // Microsecond timestamp of this azimuth
  double azimuth = 0.0;          // Azimuth angle in radians
  int azimuth_idx = 0;           // Index of this azimuth in the raw scan
};

// A Doppler scan composed of multiple azimuths
using DopplerScan = std::vector<DopplerAzimuth>;
}  // namespace srd::common