#ifndef STELLA_VSLAM_FEATURE_SIFT_PARAMS_H
#define STELLA_VSLAM_FEATURE_SIFT_PARAMS_H

#include <nlohmann/json_fwd.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "stella_vslam/feature/params.h"

namespace stella_vslam {
namespace feature {
struct sift_params : public params {
    sift_params() = delete;

    sift_params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels, const unsigned int num_sublevels, const double threshold, const double edge_threshold);
    sift_params(const YAML::Node& yaml_node);
    sift_params(const nlohmann::json& json_obj);

    ~sift_params() = default;

    unsigned int num_sublevels_;
    double threshold_;
    double edge_threshold_;

    nlohmann::json to_json() const override;
};
} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_SIFT_PARAMS_H
