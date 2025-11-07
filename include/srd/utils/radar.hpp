#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>
#include <string>

namespace srd::utils {

// Load the timestamps, azimuths, up_chirp labels, and FFT data from raw radar data
// Assumes the first 11 bytes of each row are metadata:
// - bytes 0-7: int64_t timestamp
// - bytes 8-9: uint16_t azimuth
// - byte 10: bool up_chirp
// The remaining bytes are FFT magnitude data (uint8_t), one per range bin
void load_radar(const cv::Mat& raw_data,
                std::vector<int64_t>& timestamps,
                std::vector<double>& azimuths,
                std::vector<bool>& up_chirps,
                cv::Mat& fft_data);

// Extract the microsecond timestamp for a radar frame from its path
// Assumes the filename is formatted as "<timestamp>.png"
int64_t get_stamp_from_path(const std::string &path);

} // namespace srd::utils