#include "stella_vslam/feature/orb_params.h"

#include <nlohmann/json.hpp>
#include <iostream>

namespace stella_vslam {
namespace feature {

orb_params::orb_params(const std::string& name)
    : orb_params(name, 1.2, 8, 20, 7) {}

orb_params::orb_params(const std::string& name, const float scale_factor, const unsigned int num_levels,
                       const unsigned int ini_fast_thr, const unsigned int min_fast_thr)
    : params(name, scale_factor, num_levels, ini_fast_thr, min_fast_thr) {
    scale_factors_ = calc_scale_factors(num_levels_, scale_factor_);
    inv_scale_factors_ = calc_inv_scale_factors(num_levels_, scale_factor_);
    level_sigma_sq_ = calc_level_sigma_sq(num_levels_, scale_factor_);
    inv_level_sigma_sq_ = calc_inv_level_sigma_sq(num_levels_, scale_factor_);
}

orb_params::orb_params(const YAML::Node& yaml_node)
    : orb_params(yaml_node["name"].as<std::string>("default ORB feature extraction setting"),
                 yaml_node["scale_factor"].as<float>(1.2),
                 yaml_node["num_levels"].as<unsigned int>(8),
                 yaml_node["ini_fast_threshold"].as<unsigned int>(20),
                 yaml_node["min_fast_threshold"].as<unsigned int>(7)) {}

nlohmann::json orb_params::to_json() const {
    return {{"name", name_},
            {"scale_factor", scale_factor_},
            {"num_levels", num_levels_},
            {"ini_fast_threshold", ini_fast_thr_},
            {"min_fast_threshold", min_fast_thr_}};
}

std::ostream& operator<<(std::ostream& os, const orb_params& oparam) {
    os << "- scale factor: " << oparam.scale_factor_ << std::endl;
    os << "- number of levels: " << oparam.num_levels_ << std::endl;
    os << "- initial fast threshold: " << oparam.ini_fast_thr_ << std::endl;
    os << "- minimum fast threshold: " << oparam.min_fast_thr_ << std::endl;
    return os;
}

} // namespace feature
} // namespace stella_vslam
