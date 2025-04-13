#ifndef STELLA_VSLAM_FEATURE_AKAZE_EXTRACTOR_H
#define STELLA_VSLAM_FEATURE_AKAZE_EXTRACTOR_H

#include "stella_vslam/feature/extractor.h"
#include "stella_vslam/feature/akaze_params.h"
#include <opencv2/features2d.hpp>

namespace stella_vslam {
namespace feature {
class akaze_extractor : public extractor {
private:
    const akaze_params* params_;
    cv::KAZE::DiffusivityType get_diffusivity(std::string diffusity);

public:
    akaze_extractor() = delete;
    akaze_extractor(const akaze_params* params);
    ~akaze_extractor() = default;

    //! Extract keypoints and each descriptor of them
    void extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                 std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors) override;
};
} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_AKAZE_EXTRACTOR_H
