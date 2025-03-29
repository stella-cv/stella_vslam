#include "stella_vslam/feature/sift_params.h"

#include <nlohmann/json.hpp>
#include <iostream>

namespace stella_vslam {
namespace feature {
nlohmann::json sift_params::to_json() const {
    return {{"name", name_},
            {"scale_factor", scale_factor_},
            {"num_levels", num_levels_},
            {"ini_fast_threshold", ini_fast_thr_},
            {"min_fast_threshold", min_fast_thr_}};
}
} // namespace feature
} // namespace stella_vslam