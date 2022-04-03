#ifndef OPENVSLAM_DATA_MARKER2D_H
#define OPENVSLAM_DATA_MARKER2D_H

#include "openvslam/type.h"
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

namespace openvslam {
namespace data {

class marker2d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    marker2d(const std::vector<cv::Point2f>& undist_corners, const eigen_alloc_vector<Vec3_t>& bearings,
             const Mat33_t& rot_cm, const Vec3_t& trans_cm, unsigned int id);

    //! Compute corner positions on the world from camera pose and corner positions on the camera
    eigen_alloc_vector<Vec3_t> compute_corners_pos_w(const Mat44_t& cam_pose_wc, const eigen_alloc_vector<Vec3_t>& corners_pos) const;

    //! undistorted corner points
    std::vector<cv::Point2f> undist_corners_;

    //! bearing of corners
    eigen_alloc_vector<Vec3_t> bearings_;

    //! marker pose (camera -> marker)
    Mat33_t rot_cm_;
    Vec3_t trans_cm_;

    //! marker ID
    unsigned int id_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_MARKER2D_H
