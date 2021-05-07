#include "openvslam/feature/orb_params.h"

#include <iostream>

namespace openvslam {
namespace feature {

orb_params::orb_params(const unsigned int max_num_keypts, const unsigned int ini_max_num_keypts,
                       const float scale_factor, const unsigned int num_levels,
                       const unsigned int ini_fast_thr, const unsigned int min_fast_thr,
                       const std::vector<std::vector<float>>& mask_rects)
    : max_num_keypts_(max_num_keypts), ini_max_num_keypts_(ini_max_num_keypts),
      scale_factor_(scale_factor), num_levels_(num_levels),
      ini_fast_thr_(ini_fast_thr), min_fast_thr(min_fast_thr),
      mask_rects_(mask_rects) {
    for (const auto& v : mask_rects_) {
        if (v.size() != 4) {
            throw std::runtime_error("mask rectangle must contain four parameters");
        }
        if (v.at(0) >= v.at(1)) {
            throw std::runtime_error("x_max must be greater than x_min");
        }
        if (v.at(2) >= v.at(3)) {
            throw std::runtime_error("y_max must be greater than x_min");
        }
    }
}

orb_params::orb_params(const YAML::Node& yaml_node)
    : orb_params(yaml_node["max_num_keypoints"].as<unsigned int>(2000),
                 yaml_node["ini_max_num_keypoints"].as<unsigned int>(4000),
                 yaml_node["scale_factor"].as<float>(1.2),
                 yaml_node["num_levels"].as<unsigned int>(8),
                 yaml_node["ini_fast_threshold"].as<unsigned int>(20),
                 yaml_node["min_fast_threshold"].as<unsigned int>(7),
                 yaml_node["mask_rectangles"].as<std::vector<std::vector<float>>>(std::vector<std::vector<float>>())) {}

std::vector<float> orb_params::calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor) {
    std::vector<float> scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factors.at(level) = scale_factor * scale_factors.at(level - 1);
    }
    return scale_factors;
}

std::vector<float> orb_params::calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor) {
    std::vector<float> inv_scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        inv_scale_factors.at(level) = (1.0f / scale_factor) * inv_scale_factors.at(level - 1);
    }
    return inv_scale_factors;
}

std::vector<float> orb_params::calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor) {
    float scale_factor_at_level = 1.0;
    std::vector<float> level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        level_sigma_sq.at(level) = scale_factor_at_level * scale_factor_at_level;
    }
    return level_sigma_sq;
}

std::vector<float> orb_params::calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor) {
    float scale_factor_at_level = 1.0;
    std::vector<float> inv_level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        inv_level_sigma_sq.at(level) = 1.0f / (scale_factor_at_level * scale_factor_at_level);
    }
    return inv_level_sigma_sq;
}

std::ostream& operator<<(std::ostream& os, const orb_params& oparam) {
    os << "- number of keypoints: " << oparam.max_num_keypts_ << std::endl;
    os << "- initial number of keypoints: " << oparam.ini_max_num_keypts_ << std::endl;
    os << "- scale factor: " << oparam.scale_factor_ << std::endl;
    os << "- number of levels: " << oparam.num_levels_ << std::endl;
    os << "- initial fast threshold: " << oparam.ini_fast_thr_ << std::endl;
    os << "- minimum fast threshold: " << oparam.min_fast_thr << std::endl;
    if (!oparam.mask_rects_.empty()) {
        os << "- mask rectangles:" << std::endl;
        for (const auto& mask_rect : oparam.mask_rects_) {
            os << "  - ["
               << mask_rect.at(0) << ", "
               << mask_rect.at(1) << ", "
               << mask_rect.at(2) << ", "
               << mask_rect.at(3) << "]" << std::endl;
        }
    }
    return os;
}

} // namespace feature
} // namespace openvslam
