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

    sift_params(std::string name, float scale_factor, unsigned int num_levels, unsigned int num_sublevels, double threshold, double edge_threshold);
    sift_params(const YAML::Node& yaml_node);

    ~sift_params() = default;

    unsigned int num_sublevels_;
    double threshold_;
    double edge_threshold_;

    nlohmann::json to_json() const override;
};
} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_SIFT_PARAMS_H
