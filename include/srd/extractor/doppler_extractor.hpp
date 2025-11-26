#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "srd/common/doppler_types.hpp"
#include <optional>

namespace srd::extractor {

using srd::common::DopplerScan;

class DopplerExtractor {
 public:
  using Ptr = std::shared_ptr<DopplerExtractor>;
  using ConstPtr = std::shared_ptr<const DopplerExtractor>;

  struct Options {
    /// Radar hardware parameters
    double radar_res = 0.04381;     // meters/pixel
    long double f_t = 75.04e9;      // Hz
    double meas_freq = 1600.0;      // Hz
    long double del_f = 893.0e6;    // Hz

    // Doppler extraction parameters
    double min_range = 5;              // meters
    double max_range = 150;            // meters
    double beta_corr_fact = 0.954;  // m/s
    int pad_num = 50;
    double max_velocity = 50.0;     // m/s
    // Filtering parameters
    double sigma_gauss = 17;
    double z_q = 2.5;
    // Ransac parameters
    int ransac_max_iter = 100;
    double ransac_threshold = 6.0;
    double ransac_prior_threshold = 1.0;

    // Velocity estimation parameters
    int opt_max_iter = 20;
    double opt_threshold = 1e-3;
    double cauchy_rho = 4.0;
    // Bias correction parameters
    double x_bias_slope = 0.0;
    double x_bias_intercept = 0.0;
    double y_bias_slope = 0.0;
    double y_bias_intercept = 0.0;
  };

  DopplerExtractor();
  explicit DopplerExtractor(const Options& options);

  void extract_doppler(const cv::Mat& fft_data,
                       const std::vector<double>& azimuths,
                       const std::vector<int64_t>& timestamps,
                       const std::vector<bool>& chirps,
                       DopplerScan& doppler_scan) const;
  void cen_filter(cv::Mat& signal, double sigma_gauss, double z_q) const;

  void ransac_scan(DopplerScan& doppler_scan,
                   std::optional<Eigen::Vector2d> prior_model = std::nullopt) const;

  Eigen::Vector2d register_scan(const DopplerScan &doppler_scan,
                                std::optional<Eigen::Vector2d> varpi_prior = std::nullopt) const;

  // Combined method to get ego velocity from FFT data directly
  Eigen::Vector2d get_ego_velocity(const cv::Mat& fft_data,
                       const std::vector<double>& azimuths,
                       const std::vector<int64_t>& timestamps,
                       const std::vector<bool>& chirps,
                       const bool use_ransac,
                       DopplerScan& doppler_scan) const;

 private:
  Options options_;
};

}  // namespace srd::extractor
