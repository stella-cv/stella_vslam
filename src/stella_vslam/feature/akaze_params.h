#ifndef STELLA_VSLAM_FEATURE_AKAZE_PARAMS_H
#define STELLA_VSLAM_FEATURE_AKAZE_PARAMS_H

#include <nlohmann/json_fwd.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

#include "stella_vslam/feature/params.h"

namespace stella_vslam {
namespace feature {
struct akaze_params : params {
    akaze_params() = delete;

    akaze_params(const YAML::Node& yaml_node);
    akaze_params(std::string name, unsigned int num_levels, unsigned int num_sublevels, double threshold, std::string diffusivity);

    ~akaze_params() = default;

    unsigned int num_sublevels_;
    float threshold_;
    std::string diffusivity_;

    nlohmann::json to_json() const override;
};

} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_AKAZE_PARAMS_H
