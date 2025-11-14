#include "srd/extractor/doppler_extractor.hpp"
#include <opencv2/imgproc.hpp>
#include <random>
#include <numeric>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace srd::extractor {

DopplerExtractor::DopplerExtractor() : options_() {
  init_gaussian_kernel();
}

DopplerExtractor::DopplerExtractor(const Options& options) : options_(options) {
  init_gaussian_kernel();
}

void DopplerExtractor::init_gaussian_kernel() {
  // Make sure sigma is sensible, throw error if not
  int sigma = options_.sigma_gauss;
  if (sigma <= 0) {
    throw std::invalid_argument("[DopplerExtractor] sigma_gauss must be positive.");
  }

  // ensure odd window size, at least 3
  int fsize = std::max(3, (sigma * 6) | 1);

  gauss_kernel_ = cv::getGaussianKernel(fsize, sigma, CV_32F).t();
}


void DopplerExtractor::extract_doppler(
    const cv::Mat& fft_data,
    const std::vector<double>& azimuths,
    const std::vector<int64_t>& timestamps,
    const std::vector<bool>& chirps,
    DopplerScan& doppler_scan) const {
  
  // --- Input validation ---
  if (fft_data.empty()) {
    throw std::invalid_argument("[extract_doppler] Empty FFT data.");
  }
  if (azimuths.size() != timestamps.size() || azimuths.size() != chirps.size()) {
    throw std::invalid_argument("[extract_doppler] Input vectors must have equal size.");
  }

  const int N = fft_data.rows;
  const int range_bins = fft_data.cols;

  // --- Parameters ---
  const double radar_res = options_.radar_res;
  const double beta_up = options_.beta_corr_fact * options_.f_t / (options_.del_f * options_.meas_freq);
  const double beta_down = -beta_up;
  const int pad_num = options_.pad_num;
  int max_range = (options_.max_range > 0) ? options_.max_range : static_cast<int>(range_bins * radar_res);
  const int min_range_pix = options_.min_range / radar_res;
  const int max_range_pix = max_range / radar_res;

  // --- Prepare reusable variables ---
  doppler_scan.clear();
  doppler_scan.reserve(N);
  int W = max_range_pix - min_range_pix;
  cv::Mat az_i(1, W, CV_32F);
  cv::Mat az_i1(1, W, CV_32F);
  cv::Mat padded(1, W + 2 * pad_num, CV_32F);
  cv::Mat corr(1, W + pad_num * 2 - W + 1, CV_32F);
  cv::Point max_idx;

  // --- Process successive azimuths ---
  // TOOD: Change cv:Mat to cv:Mat1f
  fft_data.row(0).colRange(min_range_pix, max_range_pix).copyTo(az_i);
  cen_filter(az_i, options_.z_q);
  for (int i = 0; i < N - 1; ++i) {
    fft_data.row(i+1).colRange(min_range_pix, max_range_pix).copyTo(az_i1);
    cen_filter(az_i1, options_.z_q);

    if (cv::countNonZero(az_i) == 0 || cv::countNonZero(az_i1) == 0)
      continue;

    padded.setTo(0);
    az_i1.copyTo(padded.colRange(pad_num, pad_num + az_i.cols));

    cv::matchTemplate(az_i, padded, corr, cv::TM_CCORR_NORMED);
    cv::minMaxLoc(corr, nullptr, nullptr, nullptr, &max_idx);

    const double del_r = (max_idx.x - pad_num) / 2.0;
    double u_val = del_r * radar_res / (chirps[i] ? beta_up : beta_down);
    if (std::abs(u_val) > options_.max_velocity)
      continue;

    double u_az = (azimuths[i] + azimuths[i + 1]) / 2.0;
    int64_t u_time = static_cast<int64_t>(((timestamps[i] + timestamps[i + 1]) / 2.0));

    doppler_scan.push_back({u_val, u_time, u_az, i});
    az_i = std::move(az_i1);
  }
}

void DopplerExtractor::cen_filter(cv::Mat& signal, int z_q) const {
  if (signal.empty()) return;

  // Subtract mean
  cv::Scalar mean_val = cv::mean(signal);
  cv::Mat q = signal - mean_val[0];

  // Apply Gaussian filter
  cv::Mat p;
  cv::filter2D(q, p, -1, gauss_kernel_, cv::Point(-1,-1), 0, cv::BORDER_REFLECT101);

  // Estimate noise sigma
  double sigma_q = 0;
  int count = 0;
  const float* qptr = q.ptr<float>();
  for (int i = 0; i < q.cols; ++i) {
    float v = qptr[i];
    if (v < 0) {
      sigma_q += 2.0 * v * v;
      ++count;
    }
  }
  sigma_q = (count > 0) ? 0.1 * std::sqrt(sigma_q / count) : 0.034;
  const double threshold = z_q * sigma_q;

  // Compute adaptive weighting
  cv::Mat pow_p, pow_qp, npp, nqp;
  cv::pow(p / sigma_q, 2, pow_p);
  cv::pow((q - p) / sigma_q, 2, pow_qp);
  cv::exp(-0.5 * pow_p, npp);
  cv::exp(-0.5 * pow_qp, nqp);

  cv::Mat y = q.mul(1 - nqp) + p.mul(nqp - npp);
  cv::Mat mask = (y > threshold);
  mask.convertTo(mask, CV_32F, 1.0 / 255.0);
  signal = y.mul(mask);
}

void DopplerExtractor::ransac_scan(DopplerScan& doppler_scan,
                                  std::optional<Eigen::Vector2d> prior_model) const {
  if (doppler_scan.size() < 2) {
    std::cerr << "[RANSAC] Not enough points to fit model.\n";
    return;
  }

  const int N = static_cast<int>(doppler_scan.size());
  Eigen::MatrixXd A(N, 2);
  Eigen::VectorXd b(N);

  for (int i = 0; i < N; ++i) {
    b(i) = doppler_scan[i].radial_velocity;
    A(i, 0) = std::cos(doppler_scan[i].azimuth);
    A(i, 1) = std::sin(doppler_scan[i].azimuth);
  }

  static thread_local std::mt19937 rng(99);
  std::uniform_int_distribution<int> dist(0, N - 1);

  int best_inliers = 0;
  bool use_prior = prior_model.has_value();
  Eigen::Vector2d prior_model_val = prior_model.value_or(Eigen::Vector2d::Zero());
  Eigen::Vector2d best_model = prior_model_val;
  std::vector<int> best_mask(N, 0);
  std::vector<int> mask(N, 0);

  for (int iter = 0; iter < options_.ransac_max_iter; ++iter) {
    int i1 = dist(rng), i2 = dist(rng);
    while (i2 == i1) i2 = dist(rng);

    Eigen::Matrix2d A_sample;
    Eigen::Vector2d b_sample;
    A_sample.row(0) = A.row(i1);
    A_sample.row(1) = A.row(i2);
    b_sample << b(i1), b(i2);

    Eigen::Vector2d model = A_sample.colPivHouseholderQr().solve(b_sample);

    if (use_prior && (model - prior_model_val).norm() > options_.ransac_prior_threshold)
      continue;

    Eigen::VectorXd residuals = A * model - b;
    mask.assign(N, 0);
    int num_inliers = 0;

    for (int i = 0; i < N; ++i) {
      if (std::abs(residuals(i)) < options_.ransac_threshold) {
        mask[i] = 1;
        ++num_inliers;
      }
    }

    if (num_inliers > best_inliers) {
      best_inliers = num_inliers;
      best_mask = mask;
      best_model = model;
    }
  }

  if (best_inliers < 10) {
    std::cerr << "[RANSAC] Too few inliers, using prior model.\n";
    best_model = prior_model_val;
  }

  DopplerScan filtered;
  filtered.reserve(N);
  for (int i = 0; i < N; ++i)
    if (best_mask[i]) filtered.push_back(doppler_scan[i]);

  doppler_scan.swap(filtered);
}

Eigen::Vector2d DopplerExtractor::register_scan(const DopplerScan &doppler_scan,
  std::optional<Eigen::Vector2d> varpi_prior) const {
  // Load in parameters for easy reference
  int num_meas = doppler_scan.size();

  // Initialize doppler measurement residual and A/b matrices for least squares
  Eigen::VectorXd residuals = Eigen::VectorXd::Zero(num_meas);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_meas, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_meas);
  // Fill in A and b matrices (these will be constant throughout iterations)
  for (int i = 0; i < num_meas; ++i) {
    double azimuth = doppler_scan[i].azimuth;
    A(i, 0) = std::cos(azimuth);
    A(i, 1) = std::sin(azimuth);
    b(i) = doppler_scan[i].radial_velocity;
  }

  // Initialize variables for least squares
  Eigen::VectorXd w_inv_diag = Eigen::VectorXd::Ones(num_meas);
  Eigen::VectorXd cauchy_w = Eigen::VectorXd::Ones(num_meas);
  Eigen::VectorXd varpi_prev = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd varpi_curr = Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(2, 2);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(2);

  // Load in initial guess to be used for the first iteration
  bool prior_provided = varpi_prior.has_value();
  if (prior_provided) {
    varpi_prev = varpi_prior.value();
    varpi_curr = varpi_prior.value();
  } else {
    // Start from zero velocity if no prior provided
    varpi_prev = Eigen::Vector2d::Zero();
    varpi_curr = Eigen::Vector2d::Zero();
  }

  // run least squares on Ax = b
  for (int iter = 0; iter < options_.opt_max_iter; ++iter) {
    // compute new cauchy weights
    residuals = A * varpi_curr - b;
    cauchy_w = 1.0 / (1.0 + ( residuals.array() / options_.cauchy_rho ).square());
    w_inv_diag = cauchy_w.array();

    lhs = A.transpose() * w_inv_diag.asDiagonal() * A;
    rhs = A.transpose() * w_inv_diag.asDiagonal() * b;

    // solve
    varpi_prev = varpi_curr;
    varpi_curr = lhs.inverse() * rhs;
    
    // Check for convergence
    if ((varpi_curr - varpi_prev).norm() < options_.opt_threshold) {
      break;
    }
  }

  // Only correct bias in x if we're confident we're moving
  // This bias is calibrated only for x_vel > 0.2
  if (std::abs(varpi_curr(0)) > 0.2) {
    varpi_curr(0) = varpi_curr(0) + varpi_curr(0) * options_.x_bias_slope + options_.x_bias_intercept;
  }
  varpi_curr(1) = varpi_curr(1) + varpi_curr(1) * options_.y_bias_slope + options_.y_bias_intercept;

  return varpi_curr;
}


Eigen::Vector2d DopplerExtractor::get_ego_velocity(const cv::Mat& fft_data,
                       const std::vector<double>& azimuths,
                       const std::vector<int64_t>& timestamps,
                       const std::vector<bool>& chirps,
                       const bool use_ransac,
                       DopplerScan& doppler_scan) const {
  extract_doppler(fft_data, azimuths, timestamps, chirps, doppler_scan);
  if (use_ransac) ransac_scan(doppler_scan);
  return register_scan(doppler_scan);
}

}  // namespace srd::extractor
