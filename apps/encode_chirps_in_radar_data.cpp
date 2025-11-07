
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <Eigen/Dense>
#include <cmath>
#include <cassert>
#include "yaml-cpp/yaml.h"
#include "lgmath.hpp"
#include "srd/utils/radar.hpp"
#include "srd/extractor/doppler_extractor.hpp"
#include "srd/extractor/doppler_config.hpp"

using srd::common::DopplerScan;
namespace fs = std::filesystem;

int main(int argc, char** argv)
{
    // Check if data is passed
    if (argc < 4)
    {
        std::cerr << "Please provide a data path, output path, and options path." << std::endl;
        return 1;
    }
    std::cout << "Input path: " << argv[1] << std::endl;
    std::cout << "Output name: " << argv[2] << std::endl;
    std::cout << "Options path: " << argv[3] << std::endl;
    
    // Load in paths
    const fs::path input_dir = argv[1];
    const fs::path doppler_output_dir = argv[2];

    // If output dir doesnt exist, create it
    if (!fs::exists(doppler_output_dir)) fs::create_directory(doppler_output_dir);

    std::cout << "Processing data from " << input_dir << std::endl;

    // Load in radar scans
    std::vector<fs::directory_entry> files;
    for (const auto &dir_entry : fs::directory_iterator{input_dir})
    if (!fs::is_directory(dir_entry)) files.push_back(dir_entry);
    // Select only .png files
    files.erase(std::remove_if(files.begin(), files.end(), [](const fs::directory_entry &entry) {
        return entry.path().extension() != ".png";
    }), files.end());
    std::sort(files.begin(), files.end());
    std::cout << "Found " << files.size() << " radar scans." << std::endl;

    // load yaml config
    std::string config_path = argv[3];
    std::cout << "Loading config from " << config_path << std::endl;
    YAML::Node config = YAML::LoadFile(argv[3]);
    auto opts = srd::extractor::load_doppler_options(config);
    std::cout << "Config loaded successfully" << std::endl;

    // Create extractor
    srd::extractor::DopplerExtractor extractor(opts);
    std::cout << "Doppler extractor created successfully" << std::endl;

    // Set up variables to store radar data
    std::vector<int64_t> timestamps;
    std::vector<double> azimuths;
    cv::Mat fft_data;

    // Load in what we believe initial chirp type is
    auto up_are_even = true;
    int frame = 0;

    // Create odom pipeline
    auto it = files.begin();
    int num_gt_above_0_p_5 = 0;
    Eigen::Vector2d v_est = Eigen::Vector2d::Zero();
    while (it != files.end()) {
        const auto timestamp = srd::utils::get_stamp_from_path(it->path().string());
        auto scan = cv::imread(it->path().string(), cv::IMREAD_GRAYSCALE);
        std::cout << "Loading radar frame " << frame << " with timestamp " << timestamp << std::endl;
        
        // Load in unprocessed radar data
        std::vector<bool> up_chirps;
        srd::utils::load_radar(scan, timestamps, azimuths, up_chirps, fft_data);

        // Populate chirp types based on previous guess
        std::vector<bool> dummy_chirps = std::vector<bool>(azimuths.size(), false);
        for (size_t i = 0; i < azimuths.size(); i++) {
            if (i % 2 == 0) {
                dummy_chirps[i] = up_are_even;
            } else {
                dummy_chirps[i] = !up_are_even;
            }
        }

        // Populate dummy timestamps since they're all just 0 for now. Let's evenly space
        // 250ms from start to finish with timestamp being the center
        int64_t start_time = timestamp - 125000;
        int64_t end_time = timestamp + 125000;
        int64_t time_step = (end_time - start_time) / static_cast<int64_t>(azimuths.size());
        for (size_t i = 0; i < azimuths.size(); i++) {
            timestamps[i] = start_time + i * time_step;
        }

        // Preprocess radar data into doppler scan
        DopplerScan doppler_scan;
        extractor.extract_doppler(fft_data, azimuths, timestamps, dummy_chirps, doppler_scan);

        // Run RANSAC to filter out outliers
        extractor.ransac_scan(doppler_scan);
        if (doppler_scan.size() < 10) {
            std::cerr << "Frame " << frame << " has less than 10 points!! Something is wrong..." << std::endl;
            break;
        }
        
        // Get velocity estimate
        v_est = extractor.register_scan(doppler_scan);

        // Extract velocity estimate from odom
        std::cout << "Velocity estimate: (" << v_est[0] << ", " << v_est[1] << ") at time " << timestamp << std::endl;

        // If fwd velocity is too small, no flip should happen
        if (abs(v_est(0)) > 1.0) {
            num_gt_above_0_p_5 += 1;
            // If forward velocity is negative, then we likely have the chirp types flipped
            if (v_est[0] < 0.0) {
                std::cout << "\033[1;31mFlipping chirp type at frame " << frame
                << " with estimated velocity (" << v_est[0] << ", " << v_est[1] << ")\033[0m" << std::endl;
                up_are_even = !up_are_even;

                // Decide how far to go back to recompute with correct chirp type
                if (num_gt_above_0_p_5 < 10) {
                    // If this is just the start of motion in the sequence, then we messed up the initial guess
                    std::cout << "\033[1;31mStart of movement. Likely an error in guess of initial flip type. Going all the way back to start.\033[0m" << std::endl;
                    frame = 0;
                    num_gt_above_0_p_5 = 0;
                    it = files.begin();
                }
                // Else, we just need to redo the last frame since it generated a flipped chirp type
                continue;
            }
        }
        
        // Populate chirp type
        for (size_t i = 0; i < azimuths.size(); i++) {
            if (i % 2 == 0) {
                scan.at<uchar>(i, 10) = up_are_even * 255.0;
            } else {
                scan.at<uchar>(i, 10) = !up_are_even * 255.0;
            }
        }
        // Save the processed radar data
        cv::imwrite((doppler_output_dir / it->path().filename()).string(), scan);
        it++;
        frame++;
    }

    std::cout << "Finished processing all radar scans." << std::endl;
    return 0;
}