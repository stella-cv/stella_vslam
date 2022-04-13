#ifndef STELLA_VSLAM_MARKER_DETECTOR_BASE_H
#define STELLA_VSLAM_MARKER_DETECTOR_BASE_H

#include <unordered_map>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

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

class base {
public:
    //! Constructor
    base(const camera::base* camera,
         const std::shared_ptr<marker_model::base>& marker_model,
         unsigned int num_iter = 10,
         double reproj_error_threshold = 0.1);

    virtual ~base() = default;

    //! Detect markers on image
    virtual void detect_2d(const cv::_InputArray& in_image, std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids) const = 0;

    //! Detect markers and create marker2d
    void detect(const cv::_InputArray& in_image, std::unordered_map<unsigned int, data::marker2d>& markers_2d) const;

    //! Return true if aruco is enable
    static bool is_valid();

    //! camera
    const camera::base* camera_;
    //! marker model
    std::shared_ptr<marker_model::base> marker_model_ = nullptr;

    //! iterations of pnp_solver
    unsigned int num_iter_ = 10;

    //! reprojection error threshold
    double reproj_error_threshold_ = 0.1;
};

} // namespace marker_detector
} // namespace stella_vslam

#endif // STELLA_VSLAM_MARKER_DETECTOR_BASE_H
