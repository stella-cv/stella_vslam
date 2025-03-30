#ifndef STELLA_VSLAM_FEATURE_PARAMS_H
#define STELLA_VSLAM_FEATURE_PARAMS_H
#include <nlohmann/json_fwd.hpp>
#include <yaml-cpp/yaml.h>
#include "stella_vslam/feature/default_params.h"
namespace stella_vslam {
namespace feature {
struct params {
    params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels);
    params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels,
           const unsigned int ini_fast_thr, const unsigned int min_fast_thr);
    virtual ~params() = default;

    //! name (id for saving)
    const std::string name_;
    const std::string type_;

    const float scale_factor_ = default_scale_factor;
    const float log_scale_factor_ = default_log_scale_factor;
    const unsigned int num_levels_ = default_num_levels;
    const unsigned int ini_fast_thr_ = default_ini_fast_thr;
    const unsigned int min_fast_thr_ = default_min_fast_thr;

    //! A list of the scale factor of each pyramid layer
    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;

    //! A list of the sigma of each pyramid layer
    std::vector<float> level_sigma_sq_;
    std::vector<float> inv_level_sigma_sq_;

    //! Calculate scale factors
    static std::vector<float> calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of scale factors
    static std::vector<float> calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate squared sigmas at all levels
    static std::vector<float> calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of squared sigmas at all levels
    static std::vector<float> calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

    virtual nlohmann::json to_json() const = 0;
};
} // namespace feature
} // namespace stella_vslam
#endif // STELLA_VSLAM_FEATURE_PARAMS_H
