#pragma once
#include <vector>
#include <cstdint>

namespace srd::common {
// Represents one radar azimuth measurement
struct DopplerAzimuth {
  double radial_velocity{};  // Radial velocity of the point
  int64_t timestamp{};         // Microsecond timestamp of this azimuth
  double azimuth{};          // Azimuth angle in radians
  int azimuth_idx{};           // Index of this azimuth in the raw scan
};

// A Doppler scan composed of multiple azimuths
using DopplerScan = std::vector<DopplerAzimuth>;
}  // namespace srd::common
