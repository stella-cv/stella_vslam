#include "stella_vslam/feature/sift_params.h"
#include "stella_vslam/feature/default_params.h"

#include <nlohmann/json.hpp>
#include <iostream>

namespace stella_vslam {
namespace feature {
sift_params::sift_params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels, const unsigned int num_sublevels, const double threshold, const double edge_threshold)
    : params(name, type, scale_factor, num_levels),
      num_sublevels_(num_sublevels),
      threshold_(threshold),
      edge_threshold_(edge_threshold) {}

sift_params::sift_params(const YAML::Node& yaml_node)
    : sift_params(yaml_node["name"].as<std::string>(),
                  yaml_node["type"].as<std::string>(),
                  yaml_node["scale_factor"].as<float>(),
                  yaml_node["num_levels"].as<unsigned int>(default_num_levels),
                  yaml_node["num_sublevels"].as<unsigned int>(),
                  yaml_node["threshold"].as<double>(),
                  yaml_node["edge_threshold"].as<double>()) {}

sift_params::sift_params(const nlohmann::json& json_obj)
    : sift_params(json_obj.at("name").get<std::string>(),
                  json_obj.at("type").get<std::string>(),
                  json_obj.at("scale_factor").get<float>(),
                  json_obj.at("num_levels").get<unsigned int>(),
                  json_obj.at("num_sublevels").get<unsigned int>(),
                  json_obj.at("threshold").get<double>(),
                  json_obj.at("edge_threshold").get<double>()) {}

nlohmann::json sift_params::to_json() const {
    return {{"name", name_},
            {"type", type_},
            {"scale_factor", scale_factor_},
            {"num_levels", num_levels_},
            {"num_sublevels", num_sublevels_},
            {"edge_threshold", edge_threshold_},
            {"threshold", threshold_},
            {"ini_fast_threshold", ini_fast_thr_},
            {"min_fast_threshold", min_fast_thr_}};
}
} // namespace feature
} // namespace stella_vslam