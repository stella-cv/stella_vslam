#include "stella_vslam/feature/akaze_params.h"
#include "stella_vslam/feature/default_params.h"

#include <nlohmann/json.hpp>
#include <iostream>

namespace stella_vslam {
namespace feature {
akaze_params::akaze_params(std::string name, unsigned int num_levels, unsigned int num_sublevels, double threshold, std::string diffusivity)
    : params(name, default_scale_factor, num_levels),
      num_sublevels_{num_sublevels},
      threshold_{threshold},
      diffusivity_{std::move(diffusivity)} {}

akaze_params::akaze_params(const YAML::Node& yaml_node)
    : akaze_params(yaml_node["name"].as<std::string>(),
                   yaml_node["num_levels"].as<unsigned int>(),
                   yaml_node["num_sublevels"].as<unsigned int>(),
                   yaml_node["threshold"].as<double>(),
                   yaml_node["diffusivity"].as<std::string>()) {}

nlohmann::json akaze_params::to_json() const {
    return {{"name", name_},
            {"scale_factor", scale_factor_},
            {"num_levels", num_levels_},
            {"ini_fast_threshold", ini_fast_thr_},
            {"min_fast_threshold", min_fast_thr_}};
}
} // namespace feature
} // namespace stella_vslam