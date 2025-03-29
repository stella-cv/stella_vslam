#include "stella_vslam/feature/sift_params.h"

#include <nlohmann/json.hpp>
#include <iostream>

namespace stella_vslam {
namespace feature {
sift_params::sift_params(std::string name, float scale_factor, unsigned int num_levels, double threshold, double edge_threshold)
    : params(name, scale_factor, num_levels),
      threshold_(threshold),
      edge_threshold_(edge_threshold) {}

sift_params::sift_params(const YAML::Node& yaml_node)
    : sift_params(yaml_node["name"].as<std::string>(),
                  yaml_node["scale_factor"].as<float>(),
                  yaml_node["num_levels"].as<unsigned int>(),
                  yaml_node["threshold"].as<double>(),
                  yaml_node["edge_threshold"].as<double>()) {}
nlohmann::json sift_params::to_json() const {
    return {{"name", name_},
            {"scale_factor", scale_factor_},
            {"num_levels", num_levels_},
            {"ini_fast_threshold", ini_fast_thr_},
            {"min_fast_threshold", min_fast_thr_}};
}
} // namespace feature
} // namespace stella_vslam