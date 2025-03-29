#include "stella_vslam/feature/akaze_extractor.h"

#include <opencv2/features2d.hpp>

namespace stella_vslam {
namespace feature {
akaze_extractor::akaze_extractor(const akaze_params* params)
    : extractor(), params_(params) {}

void akaze_extractor::extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                              std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors) {
    auto get_diffusivity = [](const std::string& diffu) -> cv::KAZE::DiffusivityType {
        if (diffu == "PM_G1")
            return cv::KAZE::DiffusivityType::DIFF_PM_G1;
        else if (diffu == "PM_G2")
            return cv::KAZE::DiffusivityType::DIFF_PM_G2;
        else if (diffu == "WEICKERT")
            return cv::KAZE::DiffusivityType::DIFF_WEICKERT;
        else
            return cv::KAZE::DiffusivityType::DIFF_CHARBONNIER;
    };

    auto akaze_detector{cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, this->params_->threshold_, this->params_->num_levels_, this->params_->num_sublevels_, get_diffusivity(this->params_->diffusivity_))};
    akaze_detector->detectAndCompute(in_image, in_image_mask, keypts, out_descriptors);
}

} // namespace feature
} // namespace stella_vslam