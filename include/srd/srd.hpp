#pragma once

// Main public include for the Spinning Radar Doppler library.
// Provides access to all commonly used types and functions.

#include "srd/common/doppler_azimuth.hpp"
#include "srd/utils/utils.hpp"

namespace srd {

// Re-export important data types so users don't need to dig into ::common
using common::DopplerScan;

}  // namespace srd
