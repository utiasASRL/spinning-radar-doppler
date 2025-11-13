#pragma once

#include <yaml-cpp/yaml.h>
#include "srd/extractor/doppler_extractor.hpp"

namespace srd::extractor {

DopplerExtractor::Options load_extractor_options(const YAML::Node& config);

} // namespace srd::extractor
