#include "stella_vslam/feature/sift_extractor.h"
namespace stella_vslam {
namespace feature {
sift_extractor::sift_extractor(const sift_params* params)
    : extractor(), params_(params) {
}

void sift_extractor::extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                             std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors) {
}
} // namespace feature
} // namespace stella_vslam