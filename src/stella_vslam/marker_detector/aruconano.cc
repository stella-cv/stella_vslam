#include "stella_vslam/marker_detector/aruconano.h"
#include "stella_vslam/marker_model/aruconano.h"
#include "stella_vslam/data/marker2d.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/solve/pnp_solver.h"
#include <iostream>

namespace stella_vslam {
namespace marker_detector {
aruconano::aruconano(const camera::base* camera,
                     const std::shared_ptr<marker_model::base>& marker_model,
                     const unsigned int num_iter,
                     const double reproj_error_threshold)
    : base(camera, marker_model, num_iter, reproj_error_threshold) {
    auto aruco_marker_model = std::static_pointer_cast<marker_model::aruconano>(marker_model);
    dict_ = aruco_marker_model->dict_;
    detector_ = std::make_unique<::aruconano::MarkerDetector>();
}

void aruconano::detect_2d(const cv::_InputArray& in_image, std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids) const {
    auto markers = detector_->detect(in_image.getMat(), dict_);

    for (auto& m : markers) {
        ids.push_back(m.id);

        std::vector<cv::Point2f> new_corners;
        for (auto pt : m)
            new_corners.push_back(pt);
        corners.push_back(new_corners);
    }
}

} // namespace marker_detector
} // namespace stella_vslam
