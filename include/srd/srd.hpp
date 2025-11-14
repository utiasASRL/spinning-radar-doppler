#pragma once

// Main public include for the Spinning Radar Doppler library.
// Provides access to all commonly used types and functions.

#include "srd/common/doppler_types.hpp"
#include "srd/extractor/doppler_extractor.hpp"

namespace srd {

// Re-export important data types so users don't need to dig into ::common
using common::DopplerAzimuth;
using common::DopplerScan;
using extractor::DopplerExtractor;

}  // namespace srd
