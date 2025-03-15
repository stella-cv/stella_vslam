#ifndef STELLA_VSLAM_FEATURE_EXTRACTOR_PARAMS_H
#define STELLA_VSLAM_FEATURE_EXTRACTOR_PARAMS_H
#include <yaml-cpp/yaml.h>
namespace stella_vslam {
namespace feature {
class extractor_params {
private:
    /* data */
public:
    virtual ~extractor_params() = default;
};
} // namespace feature
} // namespace stella_vslam
#endif // STELLA_VSLAM_FEATURE_EXTRACTOR_PARAMS_H
