#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "srd/common/common.hpp"

namespace srd::extractor {

class DopplerExtractor {
 public:
  using Ptr = std::shared_ptr<DopplerExtractor>;
  using ConstPtr = std::shared_ptr<const DopplerExtractor>;

  struct Options {
    double radar_res = 0.04381;     // meters/pixel
    long double f_t = 75.04e9;      // Hz
    double meas_freq = 1600.0;      // Hz
    long double del_f = 893.0e6;    // Hz
    int min_range = 5;              // meters
    int max_range = 150;            // meters
    double beta_corr_fact = 0.954;  // m/s
    int pad_num = 50;
    int vel_dim = 2;
    int ransac_max_iter = 100;
    double ransac_threshold = 6.0;
    double ransac_prior_threshold = 1.0;
    int sigma_gauss = 17;
    int z_q = 2.5;
    double max_velocity = 50.0;     // m/s
  };

  DopplerExtractor();
  explicit DopplerExtractor(const Options& options);

  void extract_doppler(const cv::Mat& fft_data,
                       const std::vector<double>& azimuths,
                       const std::vector<int64_t>& timestamps,
                       const std::vector<bool>& chirps,
                       DopplerScan& doppler_scan) const;

  void ransac_scan(DopplerScan& doppler_scan,
                   const Eigen::Vector2d& prior_model) const;

  void cen_filter(cv::Mat& signal, int sigma_gauss, int z_q) const;

 private:
  Options options_;
};

}  // namespace srd::extractor
