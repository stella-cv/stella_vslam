#include "stella_vslam/feature/akaze_params.h"
#include "stella_vslam/feature/default_params.h"

#include <nlohmann/json.hpp>
#include <iostream>

namespace stella_vslam {
namespace feature {
akaze_params::akaze_params(const std::string& name, const std::string& type, const unsigned int num_levels, const unsigned int num_sublevels, const double threshold, const std::string& diffusivity)
    : params(name, type, default_scale_factor, num_levels),
      num_sublevels_{num_sublevels},
      threshold_{threshold},
      diffusivity_{std::move(diffusivity)} {}

akaze_params::akaze_params(const YAML::Node& yaml_node)
    : akaze_params(yaml_node["name"].as<std::string>(),
                   yaml_node["type"].as<std::string>(),
                   yaml_node["num_levels"].as<unsigned int>(),
                   yaml_node["num_sublevels"].as<unsigned int>(),
                   yaml_node["threshold"].as<double>(),
                   yaml_node["diffusivity"].as<std::string>()) {}

akaze_params::akaze_params(const nlohmann::json& json_obj)
    : akaze_params(json_obj.at("name").get<std::string>(),
                   json_obj.at("type").get<std::string>(),
                   json_obj.at("num_levels").get<unsigned int>(),
                   json_obj.at("num_sublevels").get<unsigned int>(),
                   json_obj.at("threshold").get<double>(),
                   json_obj.at("diffusivity").get<std::string>()) {}

nlohmann::json akaze_params::to_json() const {
    return {{"name", name_},
            {"type", type_},
            {"scale_factor", scale_factor_},
            {"num_levels", num_levels_},
            {"num_sublevels", num_sublevels_},
            {"diffusivity", diffusivity_},
            {"threshold", threshold_},
            {"ini_fast_threshold", ini_fast_thr_},
            {"min_fast_threshold", min_fast_thr_}};
}
} // namespace feature
} // namespace stella_vslam