#include "srd/extractor/doppler_config.hpp"

namespace srd::extractor {

DopplerExtractor::Options load_doppler_options(const YAML::Node& config) {
  DopplerExtractor::Options opts;

  auto radar = config["radar"];
  if (radar) {
    if (radar["radar_res"]) opts.radar_res = radar["radar_res"].as<double>();
    if (radar["f_t"]) opts.f_t = radar["f_t"].as<long double>();
    if (radar["meas_freq"]) opts.meas_freq = radar["meas_freq"].as<double>();
    if (radar["del_f"]) opts.del_f = radar["del_f"].as<long double>();
  }

  auto signal = config["extraction"]["signal"];
  if (signal) {
    if (signal["min_range"]) opts.min_range = signal["min_range"].as<double>();
    if (signal["max_range"]) opts.max_range = signal["max_range"].as<double>();
    if (signal["beta_corr_fact"]) opts.beta_corr_fact = signal["beta_corr_fact"].as<double>();
    if (signal["pad_num"]) opts.pad_num = signal["pad_num"].as<int>();
    if (signal["max_vel"]) opts.max_velocity = signal["max_vel"].as<double>();
  }

  auto filter = config["extraction"]["filter"];
  if (filter) {
    if (filter["sigma_gauss"]) opts.sigma_gauss = filter["sigma_gauss"].as<double>();
    if (filter["z_q"]) opts.z_q = filter["z_q"].as<double>();
  }

  auto ransac = config["extraction"]["ransac"];
  if (ransac) {
    if (ransac["max_iter"]) opts.ransac_max_iter = ransac["max_iter"].as<int>();
    if (ransac["threshold"]) opts.ransac_threshold = ransac["threshold"].as<double>();
    if (ransac["prior_threshold"]) opts.ransac_prior_threshold = ransac["prior_threshold"].as<double>();
  }

  auto velocity = config["extraction"]["velocity"];
  if (velocity) {
    if (velocity["max_iter"]) opts.opt_max_iter = velocity["max_iter"].as<int>();
    if (velocity["threshold"]) opts.opt_threshold = velocity["threshold"].as<double>();
    if (velocity["cauchy_rho"]) opts.cauchy_rho = velocity["cauchy_rho"].as<double>();
    if (velocity["x_bias_slope"]) opts.x_bias_slope = velocity["x_bias_slope"].as<double>();
    if (velocity["x_bias_intercept"]) opts.x_bias_intercept = velocity["x_bias_intercept"].as<double>();
    if (velocity["y_bias_slope"]) opts.y_bias_slope = velocity["y_bias_slope"].as<double>();
    if (velocity["y_bias_intercept"]) opts.y_bias_intercept = velocity["y_bias_intercept"].as<double>();
  }

  return opts;
}

} // namespace srd::extractor
