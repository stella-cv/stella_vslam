#ifndef STELLA_VSLAM_FEATURE_SURF_PARAMS_H
#define STELLA_VSLAM_FEATURE_SURF_PARAMS_H

#include <nlohmann/json_fwd.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace stella_vslam {
namespace feature {
class surf_params {
private:
    /* data */
public:
    surf_params() = delete;

    surf_params(const YAML::Node& yaml_node);

    ~surf_params() = default;
};

} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_SURF_PARAMS_H
