#include "stella_vslam/marker_detector/base.h"
#include "stella_vslam/data/marker2d.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/solve/pnp_solver.h"

namespace stella_vslam {
namespace marker_detector {
base::base(const camera::base* camera,
           const std::shared_ptr<marker_model::base>& marker_model,
           const unsigned int num_iter,
           const double reproj_error_threshold)
    : camera_(camera), marker_model_(marker_model), num_iter_(num_iter), reproj_error_threshold_(reproj_error_threshold) {}

bool base::is_valid() {
    return true;
}

void base::detect(const cv::_InputArray& in_image, std::unordered_map<unsigned int, data::marker2d>& markers_2d) const {
    if (in_image.empty()) {
        return;
    }

    const auto image = in_image.getMat();
    assert(image.type() == CV_8UC1);

    // Get corner positions on image
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detect_2d(image, corners, ids);

    for (unsigned int i = 0; i < corners.size(); ++i) {
        // undistort corner positions
        std::vector<cv::Point2f> undist_corners;
        camera_->undistort_points(corners[i], undist_corners);
        eigen_alloc_vector<Vec3_t> bearings;
        camera_->convert_points_to_bearings(undist_corners, bearings);

        // Get marker pose (camera -> marker) and validate it by reprojection error
        Mat33_t rot_cm;
        Vec3_t trans_cm;
        double reproj_error = solve::pnp_solver::compute_pose(bearings, marker_model_->corners_pos_, rot_cm, trans_cm, num_iter_);

        // Create marker2d (if the pose is valid)
        if (reproj_error < reproj_error_threshold_) {
            markers_2d.emplace(ids[i], data::marker2d(undist_corners, bearings, rot_cm, trans_cm, ids[i], marker_model_));
        }
    }
}
} // namespace marker_detector
} // namespace stella_vslam
