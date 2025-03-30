#include "stella_vslam/feature/params.h"

namespace stella_vslam {
namespace feature {

params::params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels)
    : name_(name), type_(type), scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor)),
      num_levels_(num_levels) {}

params::params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr)
    : name_(name), type_(type), scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor)),
      num_levels_(num_levels), ini_fast_thr_(ini_fast_thr), min_fast_thr_(min_fast_thr) {}

std::vector<float> params::calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor) {
    std::vector<float> scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factors.at(level) = scale_factor * scale_factors.at(level - 1);
    }
    return scale_factors;
}

std::vector<float> params::calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor) {
    std::vector<float> inv_scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        inv_scale_factors.at(level) = (1.0f / scale_factor) * inv_scale_factors.at(level - 1);
    }
    return inv_scale_factors;
}

std::vector<float> params::calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor) {
    float scale_factor_at_level = 1.0;
    std::vector<float> level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        level_sigma_sq.at(level) = scale_factor_at_level * scale_factor_at_level;
    }
    return level_sigma_sq;
}

std::vector<float> params::calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor) {
    float scale_factor_at_level = 1.0;
    std::vector<float> inv_level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        inv_level_sigma_sq.at(level) = 1.0f / (scale_factor_at_level * scale_factor_at_level);
    }
    return inv_level_sigma_sq;
}
} // namespace feature
} // namespace stella_vslam