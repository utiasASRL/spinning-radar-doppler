#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "srd/utils/radar.hpp"
#include "srd/extractor/doppler_extractor.hpp"
#include "srd/extractor/extractor_config.hpp"

using srd::common::DopplerScan;
namespace fs = std::filesystem;
namespace utils = srd::utils;

void write_ego_velocity_csv(const fs::path& output_file,
                            const int64_t* frame_timestamp,
                            const Eigen::Vector2d& velocity) {
  // Open file for appending
  std::ofstream ofs(output_file, std::ios::app);
  if (!ofs.is_open()) {
    throw std::runtime_error("Failed to open output CSV file: " + output_file.string());
  }

  // Write CSV header if file is empty
  if (!fs::exists(output_file) || fs::file_size(output_file) == 0) {
    ofs << "frame_timestamp,ego_velocity_x,ego_velocity_y\n";
  }

  // Set numeric formatting
  ofs << std::fixed << std::setprecision(4);
  ofs << *frame_timestamp << "," << velocity[0] << "," << velocity[1] << "\n";
}

int main(int argc, char** argv)
{
  bool verbose = false;
  bool use_ransac = true;
  bool save_csv = true;
  std::vector<std::string> positional;

  for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--verbose") {
          verbose = true;
      } else if (arg == "--no_ransac") {
          use_ransac = false;
      } else if (arg == "--no_save") {
            save_csv = false;
      } else {
          positional.push_back(arg);
      }
  }

  if (positional.size() < 3) {
      std::cerr << "Usage:\n"
                << "./generate_doppler_velocity_csv [--verbose] [--no_ransac] [--no_save] <input_dir> <output_file> <config.yaml>\n";
      return 1;
  }

  const fs::path input_dir  = positional[0];
  const fs::path output_file = positional[1];
  const fs::path config_path = positional[2];

  std::cout << "Input directory:  " << input_dir << "\n"
            << "Output file:      " << output_file << "\n"
            << "Config file:      " << config_path << "\n\n";

  if (save_csv) {
    const fs::path output_dir = output_file.parent_path();
    if (!fs::exists(output_dir)) {
        fs::create_directory(output_dir);
        std::cout << "Created output directory.\n";
    }
    if (fs::exists(output_file)) {
        fs::remove(output_file);
        std::cout << "Removed existing output file.\n";
    }
  } else {
    std::cout << "CSV saving disabled (--no_save).\n\n";
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
  int frame_idx = 0;

  std::cout << "Processing radar scans...\n";
  if (use_ransac)
      std::cout << "RANSAC filtering: ENABLED\n";
  else
      std::cout << "RANSAC filtering: DISABLED\n";

  for (const auto& file : radar_files) {
      const int64_t timestamp = utils::get_stamp_from_path(file.string());
      cv::Mat scan = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);

      if (verbose) std::cout << "Frame " << frame_idx << ": timestamp = " << timestamp << "\n";

      std::vector<bool> up_chirps;
      utils::load_radar(scan, timestamps, azimuths, up_chirps, fft_data);

      // Extract Doppler measurements
      DopplerScan doppler_scan;
      extractor.extract_doppler(fft_data, azimuths, timestamps, up_chirps, doppler_scan);
      if (use_ransac) extractor.ransac_scan(doppler_scan);

      // Extract ego velocity
      Eigen::Vector2d velocity = extractor.register_scan(doppler_scan);
      if (verbose) std::cout << "Estimated ego velocity: [" << velocity[0] << ", " << velocity[1] << "] m/s\n";

      // Write to CSV
      if (save_csv) write_ego_velocity_csv(output_file, &timestamp, velocity);

      frame_idx++;
  }

  std::cout << "\nFinished processing radar sequence.\n";
  return 0;
}