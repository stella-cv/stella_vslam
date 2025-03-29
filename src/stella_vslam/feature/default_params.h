
#ifndef STELLA_VSLAM_DEFAULT_PARAMS_H
#define STELLA_VSLAM_DEFAULT_PARAMS_H

#include <cmath>
namespace stella_vslam {
namespace feature {

constexpr float default_scale_factor{1.2f};
const float default_log_scale_factor{std::log(default_scale_factor)};
constexpr unsigned int default_num_levels{8};
constexpr unsigned int default_ini_fast_thr{20};
constexpr unsigned int default_min_fast_thr{7};

} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_DEFAULT_PARAMS_H