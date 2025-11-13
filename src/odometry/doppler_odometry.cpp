#include "srd/odometry/doppler_odometry.hpp"
#include <opencv2/imgproc.hpp>
#include <random>
#include <numeric>
#include <iostream>

namespace srd::odometry {

DopplerOdometry::DopplerOdometry() : options_() {}

DopplerOdometry::DopplerOdometry(const Options& options, const DopplerExtractor& doppler_extractor) : options_(options) {
    doppler_extractor_ = doppler_extractor;
}

} // namespace srd::odometry