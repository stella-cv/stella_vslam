#ifndef STELLA_VSLAM_FEATURE_ORB_PARAMS_H
#define STELLA_VSLAM_FEATURE_ORB_PARAMS_H

#include <nlohmann/json_fwd.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "stella_vslam/feature/params.h"

namespace stella_vslam {
namespace feature {

struct orb_params : public params {
    orb_params() = delete;

    //! Constructor
    orb_params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr);
    orb_params(const std::string& name);

    //! Constructor
    explicit orb_params(const YAML::Node& yaml_node);
    explicit orb_params(const nlohmann::json& json_obj);

    //! Destructor
    virtual ~orb_params() = default;

    nlohmann::json to_json() const override;
};

std::ostream& operator<<(std::ostream& os, const orb_params& oparam);

} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_ORB_PARAMS_H
