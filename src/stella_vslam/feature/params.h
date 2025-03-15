#ifndef STELLA_VSLAM_FEATURE_PARAMS_H
#define STELLA_VSLAM_FEATURE_PARAMS_H
#include "stella_vslam/feature/feature_type.h"
#include "stella_vslam/feature/orb_params.h"
#include "stella_vslam/feature/surf_params.h"
#include "stella_vslam/feature/sift_params.h"

namespace stella_vslam {
namespace feature {
class params {
private:
    int feature_type;
    std::shared_ptr<orb_params> orb = nullptr;
    std::shared_ptr<surf_params> surf = nullptr;
    std::shared_ptr<sift_params> sift = nullptr;
public:
    params(const YAML::Node& yaml_node);
    ~params() = default;
    get();
};
} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_PARAMS_H
