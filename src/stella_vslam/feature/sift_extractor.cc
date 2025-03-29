#include "stella_vslam/feature/sift_extractor.h"

#include <opencv2/features2d.hpp>
namespace stella_vslam {
namespace feature {
sift_extractor::sift_extractor(const sift_params* params)
    : extractor(), params_(params) {
}

void sift_extractor::extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                             std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors) {
    auto sift_detector = cv::SIFT::create(0, this->params_->num_sublevels_, this->params_->threshold_, this->params_->edge_threshold_, this->params_->scale_factor_, CV_8U);
    sift_detector->detectAndCompute(in_image, in_image_mask, keypts, out_descriptors);
}
} // namespace feature
} // namespace stella_vslam