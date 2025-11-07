#include "srd/utils/radar.hpp"
#include <filesystem>
#include <string>
#include <cmath>
#include <stdexcept>

namespace srd::utils {

void load_radar(const cv::Mat& raw_data,
                std::vector<int64_t>& timestamps,
                std::vector<double>& azimuths,
                std::vector<bool>& up_chirps,
                cv::Mat& fft_data) {
    if (raw_data.empty()) {
        throw std::invalid_argument("[load_radar] Input radar matrix is empty");
    }

    const double encoder_conversion = 2.0 * M_PI / 5600.0;
    const int64_t time_convert = 1;  // placeholder (adjust if scaling timestamps)
    const uint32_t N = raw_data.rows;
    const uint32_t M = raw_data.cols;

    timestamps.assign(N, 0);
    azimuths.assign(N, 0.0);
    up_chirps.assign(N, true);

    const uint32_t range_bins = M - 11;
    fft_data = cv::Mat::zeros(N, range_bins, CV_32F);

    for (uint32_t i = 0; i < N; ++i) {
        const uchar* row_data = raw_data.ptr<uchar>(i);
        timestamps[i] = *reinterpret_cast<const int64_t*>(row_data) * time_convert;
        azimuths[i] = *reinterpret_cast<const uint16_t*>(row_data + 8) * encoder_conversion;
        up_chirps[i] = *(row_data + 10);

        float* fft_row = fft_data.ptr<float>(i);
        for (uint32_t j = 0; j < range_bins; ++j) {
            fft_row[j] = static_cast<float>(*(row_data + 11 + j)) / 255.0f;
        }
    }
}

int64_t get_stamp_from_path(const std::string& path) {
    namespace fs = std::filesystem;

    fs::path p(path);
    auto stem = p.stem().string();  // filename without extension

    try {
        return std::stoll(stem);
    } catch (const std::invalid_argument&) {
        throw std::runtime_error("[get_stamp_from_path] Filename is not numeric: " + stem);
    } catch (const std::out_of_range&) {
        throw std::runtime_error("[get_stamp_from_path] Timestamp out of range: " + stem);
    }
}

}  // namespace srd::utils
