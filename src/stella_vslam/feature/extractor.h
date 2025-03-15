#ifndef STELLA_VSLAM_FEATURE_EXTRACTOR_H
#define STELLA_VSLAM_FEATURE_EXTRACTOR_H
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace stella_vslam {
namespace feature {
class extractor {
public:
    virtual ~extractor() = default;
    virtual void extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                         std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors)
        = 0;

    //! Image pyramid
    std::vector<cv::Mat> image_pyramid_;
protected:
    cv::Mat rect_mask_;
};
} // namespace feature
} // namespace stella_vslam
#endif // STELLA_VSLAM_FEATURE_EXTRACTOR_H