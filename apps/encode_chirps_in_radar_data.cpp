#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lgmath.hpp"
#include "srd/utils/radar.hpp"
#include "srd/extractor/doppler_extractor.hpp"
#include "srd/extractor/doppler_config.hpp"

using srd::common::DopplerScan;
namespace fs = std::filesystem;
namespace utils = srd::utils;

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "Usage:\n"
                  << "  encode_chirps_in_radar_data <input_dir> <output_dir> <config.yaml>\n";
        return 1;
    }

    const fs::path input_dir = argv[1];
    const fs::path output_dir = argv[2];
    const fs::path config_path = argv[3];

    std::cout << "Input directory:  " << input_dir << "\n"
              << "Output directory: " << output_dir << "\n"
              << "Config file:      " << config_path << "\n\n";

    if (!fs::exists(output_dir)) {
        fs::create_directory(output_dir);
        std::cout << "Created output directory.\n";
    }

    // Gather radar image files (.png)
    std::vector<fs::path> radar_files;
    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (!fs::is_directory(entry) && entry.path().extension() == ".png")
            radar_files.emplace_back(entry.path());
    }
    std::sort(radar_files.begin(), radar_files.end());

    std::cout << "Found " << radar_files.size() << " radar scans.\n";

    // Load config + construct extractor
    YAML::Node config = YAML::LoadFile(config_path);
    auto opts = srd::extractor::load_doppler_options(config);
    srd::extractor::DopplerExtractor extractor(opts);

    std::cout << "Loaded Doppler extractor configuration.\n\n";

    // Working buffers
    std::vector<int64_t> timestamps;
    std::vector<double> azimuths;
    cv::Mat fft_data;

    bool up_are_even = true;
    int stable_motion_frames = 0;
    int frame_idx = 0;

    for (const auto& file : radar_files) {
        const int64_t timestamp = utils::get_stamp_from_path(file.string());
        cv::Mat scan = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);

        std::cout << "Frame " << frame_idx << ": timestamp = " << timestamp << "\n";

        std::vector<bool> raw_chirps;
        utils::load_radar(scan, timestamps, azimuths, raw_chirps, fft_data);

        // Predict chirp direction pattern
        std::vector<bool> chirps(azimuths.size());
        for (size_t i = 0; i < chirps.size(); i++)
            chirps[i] = (i % 2 == 0) ? up_are_even : !up_are_even;

        // Synthesize timestamps evenly over ±0.125s span
        const int64_t start_time = timestamp - 125000;
        const int64_t time_step = 250000 / static_cast<int64_t>(azimuths.size());
        for (size_t i = 0; i < timestamps.size(); i++)
            timestamps[i] = start_time + i * time_step;

        // Extract Doppler measurements
        DopplerScan doppler_scan;
        extractor.extract_doppler(fft_data, azimuths, timestamps, chirps, doppler_scan);

        extractor.ransac_scan(doppler_scan);
        if (doppler_scan.size() < 10) {
            std::cerr << "Insufficient Doppler points. Stopping.\n";
            break;
        }

        Eigen::Vector2d v_est = extractor.register_scan(doppler_scan);
        std::cout << "Estimated velocity: (" << v_est[0] << ", " << v_est[1] << ")\n";

        // Adapt chirp orientation based on forward velocity sign
        // This assumes that the radar is only moving forward!!
        // TODO: Maybe move this assumption to config?
        if (std::abs(v_est[0]) > 1.0) {
            stable_motion_frames++;
            if (v_est[0] < 0.0) {
                std::cout << "\033[1;31mChirp flip detected at frame " << frame_idx << "\033[0m\n";
                up_are_even = !up_are_even;

                if (stable_motion_frames < 10) {
                    std::cout << "Reprocessing from beginning (start-of-motion ambiguity).\n\n";
                    stable_motion_frames = 0;
                    frame_idx = 0;
                    continue;
                }
                continue;
            }
        }

        // Encode chirp type into pixel channel 10
        for (size_t i = 0; i < azimuths.size(); i++)
            scan.at<uchar>(i, 10) = (i % 2 == 0 ? up_are_even : !up_are_even) ? 255 : 0;

        cv::imwrite((output_dir / file.filename()).string(), scan);
        frame_idx++;
    }

    std::cout << "\nFinished processing radar sequence.\n";
    return 0;
}