#ifndef STELLA_VSLAM_MARKER_DETECTOR_ARUCO_H
#define STELLA_VSLAM_MARKER_DETECTOR_ARUCO_H

#include "stella_vslam/marker_detector/base.h"

#include <unordered_map>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace cv {
namespace aruco {
class DetectorParameters;
class Dictionary;
} // namespace aruco
} // namespace cv

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

class aruco : public base {
public:
    //! Constructor
    aruco(const camera::base* camera,
          const std::shared_ptr<marker_model::base>& marker_model,
          unsigned int num_iter = 10,
          double reproj_error_threshold = 0.1);

    virtual ~aruco() = default;

    //! Detect markers
    void detect_2d(const cv::_InputArray& in_image, std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids) const override;

    //! Return true if aruco is enable
    static bool is_valid();

    //! parameters for marker detection
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

} // namespace marker_detector
} // namespace stella_vslam

#endif // STELLA_VSLAM_MARKER_DETECTOR_ARUCO_H
