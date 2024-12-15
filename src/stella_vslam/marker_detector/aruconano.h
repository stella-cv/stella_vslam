#ifndef STELLA_VSLAM_MARKER_DETECTOR_ARUCONANO_H
#define STELLA_VSLAM_MARKER_DETECTOR_ARUCONANO_H

#include "stella_vslam/marker_detector/base.h"

#include <unordered_map>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <aruco_nano.h>

namespace stella_vslam {

namespace data {
class marker2d;
} // namespace data

namespace camera {
class base;
} // namespace camera

namespace marker_model {
class base;
} // namespace marker_model

namespace marker_detector {

class aruconano : public base {
public:
    //! Constructor
    aruconano(const camera::base* camera,
              const std::shared_ptr<marker_model::base>& marker_model,
              unsigned int num_iter = 10,
              double reproj_error_threshold = 0.1);

    virtual ~aruconano() = default;

    //! Detect markers
    void detect_2d(const cv::_InputArray& in_image, std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids) const override;

    std::unique_ptr<::aruconano::MarkerDetector> detector_;
    int dict_;
};

} // namespace marker_detector
} // namespace stella_vslam

#endif // STELLA_VSLAM_MARKER_DETECTOR_ARUCONANO_H
