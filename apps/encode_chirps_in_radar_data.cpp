#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "srd/utils/radar.hpp"
#include "srd/extractor/doppler_extractor.hpp"
#include "srd/extractor/extractor_config.hpp"

using srd::common::DopplerScan;
namespace fs = std::filesystem;
namespace utils = srd::utils;

int main(int argc, char** argv)
{
    bool verbose = false;
    std::vector<std::string> positional;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--verbose") {
            verbose = true;
        } else {
            positional.push_back(arg);
        }
    }

    if (positional.size() < 3) {
        std::cerr << "Usage:\n"
                  << "./encode_chirps_in_radar_data [--verbose] <input_dir> <output_dir> <config.yaml>\n";
        return 1;
    }

    const fs::path input_dir  = positional[0];
    const fs::path output_dir = positional[1];
    const fs::path config_path = positional[2];

    std::cout << "Input directory:  " << input_dir << "\n"
              << "Output directory: " << output_dir << "\n"
              << "Config file:      " << config_path << "\n\n";

    if (!fs::exists(output_dir)) {
        fs::create_directory(output_dir);
        std::cout << "Created output directory.\n";
    }

    // Check if output dir is the same as input dir
    bool overwriting_files = false;
    if (fs::equivalent(input_dir, output_dir)) {
        overwriting_files = true;
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
    auto opts = srd::extractor::load_extractor_options(config);
    srd::extractor::DopplerExtractor extractor(opts);

    // Working buffers
    std::vector<int64_t> timestamps;
    std::vector<double> azimuths;
    cv::Mat fft_data;

    bool up_are_even = true;
    int stable_motion_frames = 0;
    int frame_idx = 0;
    bool chirp_dir_initialized = false;
    std::cout << "Processing radar scans...\n";
    while (frame_idx < radar_files.size()) {
        const fs::path& file = radar_files[frame_idx];
        const int64_t timestamp = utils::get_stamp_from_path(file.string());

        if (verbose) std::cout << "Frame " << frame_idx << ": timestamp = " << timestamp << "\n";

        // Load radar data
        cv::Mat scan = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);
        std::vector<bool> raw_chirps;
        utils::load_radar(scan, timestamps, azimuths, raw_chirps, fft_data);

        // Set up_are_even based on first frame
        if (!chirp_dir_initialized) {
            up_are_even = raw_chirps.size() > 0 ? raw_chirps[0] : true;
            chirp_dir_initialized = true;
        }

        // Predict chirp direction pattern
        std::vector<bool> chirps(azimuths.size());
        for (size_t i = 0; i < chirps.size(); i++)
            chirps[i] = (i % 2 == 0) ? up_are_even : !up_are_even;

        // Extract Doppler measurements
        DopplerScan doppler_scan;
        Eigen::Vector2d v_est = extractor.get_ego_velocity(fft_data, azimuths, timestamps, chirps, /*use_ransac=*/ true, doppler_scan);
        if (verbose) std::cout << "Estimated velocity: (" << v_est[0] << ", " << v_est[1] << ")\n";

        // Adapt chirp orientation based on forward velocity sign
        // This assumes that the radar is only moving forward!!
        // TODO: Maybe move this assumption to config?
        if (std::abs(v_est[0]) > 1.0) {
            stable_motion_frames++;
            if (v_est[0] < 0.0) {
                if (verbose) std::cout << "\033[1;31mChirp flip detected at frame " << frame_idx << "\033[0m\n";
                up_are_even = !up_are_even;

                if (stable_motion_frames < 10) {
                    if (verbose) std::cout << "Reprocessing from beginning (start-of-motion ambiguity).\n\n";
                    stable_motion_frames = 0;
                    frame_idx = 0;
                    continue;
                }

                // Recompute velocity to make sure its good
                for (size_t i = 0; i < chirps.size(); i++)
                    chirps[i] = (i % 2 == 0) ? up_are_even : !up_are_even;
                if (verbose) {
                    v_est = extractor.get_ego_velocity(fft_data, azimuths, timestamps, chirps, /*use_ransac=*/ true, doppler_scan);
                    if (verbose) std::cout << "Recomputed estimated velocity: (" << v_est[0] << ", " << v_est[1] << ")\n";
                }
            }
        }

        // Only save chirp info if it's different from raw_chirps that are already encoded
        // Only valid if we're overwriting files, otherwise want to save new file always
        if (raw_chirps == chirps && overwriting_files) {
            frame_idx++;
            continue;
        }

        std::cout << "Saving new frame encoding for frame: " << frame_idx << std::endl;

        // Encode chirp type into pixel channel 10
        for (size_t i = 0; i < azimuths.size(); i++)
            scan.at<uchar>(i, 10) = (i % 2 == 0 ? up_are_even : !up_are_even) ? 255 : 0;

        cv::imwrite((output_dir / file.filename()).string(), scan);
        frame_idx++;
    }

    std::cout << "\nFinished processing radar sequence.\n";
    return 0;
}