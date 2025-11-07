#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include "srd/utils/radar.hpp"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using namespace srd::utils;

TEST(RadarUtilsTest, ThrowsOnEmptyInput) {
    cv::Mat empty;
    std::vector<int64_t> timestamps;
    std::vector<double> azimuths;
    std::vector<bool> up_chirps;
    cv::Mat fft;

    EXPECT_THROW(
        load_radar(empty, timestamps, azimuths, up_chirps, fft),
        std::invalid_argument
    );
}

TEST(RadarUtilsTest, ParsesSingleRowCorrectly) {
    const int64_t fake_timestamp = 123456789;
    const uint16_t fake_azimuth_raw = 2800;
    const bool fake_up_chirp = true;
    const uint32_t cols = 7000; // arbitrary valid width
    const uint32_t rows = 1;

    // Construct a synthetic radar matrix
    cv::Mat fake_data(rows, cols, CV_8U, cv::Scalar(0));
    std::memcpy(fake_data.ptr(0), &fake_timestamp, sizeof(int64_t));
    std::memcpy(fake_data.ptr(0) + 8, &fake_azimuth_raw, sizeof(uint16_t));
    *(fake_data.ptr(0) + 10) = fake_up_chirp;

    std::vector<int64_t> timestamps;
    std::vector<double> azimuths;
    std::vector<bool> up_chirps;
    cv::Mat fft;

    load_radar(fake_data, timestamps, azimuths, up_chirps, fft);

    ASSERT_EQ(timestamps.size(), 1);
    ASSERT_EQ(azimuths.size(), 1);
    ASSERT_EQ(up_chirps.size(), 1);
    EXPECT_EQ(timestamps[0], fake_timestamp);
    EXPECT_NEAR(azimuths[0], fake_azimuth_raw * 2.0 * M_PI / 5600.0, 1e-9);
    EXPECT_EQ(up_chirps[0], fake_up_chirp);

    // Check FFT data shape
    EXPECT_EQ(fft.rows, 1);
    EXPECT_EQ(fft.cols, cols - 11);
}

TEST(RadarUtilsTest, ThrowsOnInvalidFilePath) {
    EXPECT_THROW(get_stamp_from_path("not/a/timestamp/file.txt"), std::runtime_error);
}

TEST(RadarUtilsTest, ParsesTimestampFromPath) {
    std::string path = "/some/folder/123456789.bin";
    int64_t ts = get_stamp_from_path(path);
    EXPECT_EQ(ts, 123456789);
}

TEST(RadarUtilsTest, ParsesTimestampWithExtension) {
    std::string path = "987654321.txt";
    int64_t ts = get_stamp_from_path(path);
    EXPECT_EQ(ts, 987654321);
}

TEST(RadarUtilsTest, LoadActualRadarData) {
    namespace fs = std::filesystem;
    fs::path test_dir = fs::path(__FILE__).parent_path();  // path to this .cpp file
    fs::path radar_file_path = test_dir / "test_data" / "test_frame.png";

    cv::Mat raw_data = cv::imread(radar_file_path, cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(raw_data.empty()) << "Failed to load radar data from file.";

    std::vector<int64_t> timestamps;
    std::vector<double> azimuths;
    std::vector<bool> up_chirps;
    cv::Mat fft_data;

    EXPECT_NO_THROW(
        load_radar(raw_data, timestamps, azimuths, up_chirps, fft_data)
    );

    EXPECT_GT(timestamps.size(), 0);
    EXPECT_EQ(timestamps.size(), azimuths.size());
    EXPECT_EQ(timestamps.size(), up_chirps.size());
    EXPECT_EQ(fft_data.rows, static_cast<int>(timestamps.size()));
}