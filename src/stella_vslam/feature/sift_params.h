#ifndef STELLA_VSLAM_FEATURE_SIFT_PARAMS_H
#define STELLA_VSLAM_FEATURE_SIFT_PARAMS_H

#include <nlohmann/json_fwd.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace stella_vslam {
namespace feature {
class sift_params {
private:
    /* data */
public:
    sift_params() = delete;

    sift_params(const YAML::Node& yaml_node);
    
    ~sift_params() = default;
};
} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_SIFT_PARAMS_H
