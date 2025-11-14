#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "srd/utils/radar.hpp"
#include "srd/extractor/doppler_extractor.hpp"
#include "srd/extractor/extractor_config.hpp"

using Clock = std::chrono::steady_clock;
using ms_f = std::chrono::duration<double, std::milli>;

namespace fs = std::filesystem;
namespace utils = srd::utils;
using srd::common::DopplerScan;

struct TimerStats {
    double total_ms = 0.0;
    double min_ms = 1e12;
    double max_ms = 0.0;
    int count = 0;

    void add(double ms) {
        total_ms += ms;
        min_ms = std::min(min_ms, ms);
        max_ms = std::max(max_ms, ms);
        count++;
    }

    double avg() const {
        return total_ms / std::max(1, count);
    }
};

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage:\n"
                  << "./benchmark_doppler [--max_frames N] <input_dir> <config.yaml>\n";
        return 1;
    }

    // ------------------------------------------------------------------
    // Parse args
    // ------------------------------------------------------------------
    int max_frames = -1;
    std::vector<std::string> positional;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--max_frames") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --max_frames requires an argument.\n";
                return 1;
            }
            max_frames = std::stoi(argv[++i]);
            continue;
        }

        positional.push_back(arg);
    }

    if (positional.size() < 2) {
        std::cerr << "Usage:\n"
                  << "./benchmark_doppler [--max_frames N] <input_dir> <config.yaml>\n";
        return 1;
    }

    const fs::path input_dir   = positional[0];
    const fs::path config_path = positional[1];

    std::cout << "Input directory:  " << input_dir  << "\n"
              << "Config file:      " << config_path << "\n\n";

    // ------------------------------------------------------------------
    // Gather PNG files
    // ------------------------------------------------------------------

    std::vector<fs::path> radar_files;
    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (!fs::is_directory(entry) && entry.path().extension() == ".png")
            radar_files.emplace_back(entry.path());
    }
    std::sort(radar_files.begin(), radar_files.end());

    if (max_frames > 0 && max_frames < (int)radar_files.size()) {
        radar_files.resize(max_frames);
        std::cout << "Limiting benchmark to " << max_frames << " frames.\n\n";
    }

    std::cout << "Found " << radar_files.size() << " radar scans.\n";

    // ------------------------------------------------------------------
    // Load config + create extractor
    // ------------------------------------------------------------------

    YAML::Node config = YAML::LoadFile(config_path);
    const auto opts = srd::extractor::load_extractor_options(config);
    srd::extractor::DopplerExtractor extractor(opts);

    std::vector<int64_t> timestamps;
    std::vector<double> azimuths;
    std::vector<bool> up_chirps;
    cv::Mat fft_data;

    TimerStats t_load;
    TimerStats t_extract;
    TimerStats t_ransac;
    TimerStats t_register;
    TimerStats t_full;

    std::cout << "Benchmarking...\n\n";

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------

    for (const auto& file : radar_files) {
        const int64_t timestamp = utils::get_stamp_from_path(file.string());
        cv::Mat scan = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);

        // load_radar
        auto t0 = Clock::now();
        utils::load_radar(scan, timestamps, azimuths, up_chirps, fft_data);
        auto t1 = Clock::now();
        t_load.add(ms_f(t1 - t0).count());

        DopplerScan doppler_scan;
        auto t_full_start = Clock::now();

        // extract_doppler
        t0 = Clock::now();
        extractor.extract_doppler(fft_data, azimuths, timestamps, up_chirps, doppler_scan);
        t1 = Clock::now();
        t_extract.add(ms_f(t1 - t0).count());

        // ransac_scan
        t0 = Clock::now();
        extractor.ransac_scan(doppler_scan);
        t1 = Clock::now();
        t_ransac.add(ms_f(t1 - t0).count());

        // register_scan
        t0 = Clock::now();
        Eigen::Vector2d velocity = extractor.register_scan(doppler_scan);
        t1 = Clock::now();
        t_register.add(ms_f(t1 - t0).count());

        auto t_full_end = Clock::now();
        t_full.add(ms_f(t_full_end - t_full_start).count());
    }

    // ------------------------------------------------------------------
    // Report
    // ------------------------------------------------------------------

    std::cout << "\n=== Timing Summary (milliseconds) ===\n";
    std::cout << std::fixed << std::setprecision(4);
    auto print = [&](const std::string& label, const TimerStats& s) {
        std::cout << std::setw(16) << label << ": "
                  << "avg = " << std::setw(10) << s.avg()
                  << "   min = " << std::setw(10) << s.min_ms
                  << "   max = " << std::setw(10) << s.max_ms
                  << "   (n = " << s.count << ")\n";
    };

    print("load_radar",      t_load);
    print("extract_doppler", t_extract);
    print("ransac_scan",     t_ransac);
    print("register_scan",   t_register);
    print("full pipeline",   t_full);

    std::cout << "\nDone.\n";
    return 0;
}
