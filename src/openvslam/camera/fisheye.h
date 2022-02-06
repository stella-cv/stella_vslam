#ifndef OPENVSLAM_CAMERA_FISHEYE_H
#define OPENVSLAM_CAMERA_FISHEYE_H

#include "openvslam/camera/base.h"

#include <opencv2/calib3d.hpp>

namespace openvslam {
namespace camera {

class fisheye final : public base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    fisheye(const std::string& name, const setup_type_t& setup_type, const color_order_t& color_order,
            const unsigned int cols, const unsigned int rows, const double fps,
            const double fx, const double fy, const double cx, const double cy,
            const double k1, const double k2, const double k3, const double k4,
            const double focal_x_baseline = 0.0, const double depth_thr = 0.0);

    fisheye(const YAML::Node& yaml_node);

    ~fisheye() override;

    void show_parameters() const override final;

    image_bounds compute_image_bounds() const override final;

    cv::Point2f undistort_point(const cv::Point2f& dist_pt) const override final;

    Vec3_t convert_point_to_bearing(const cv::Point2f& undist_pt) const override final;

    cv::Point2f convert_bearing_to_point(const Vec3_t& bearing) const override final;

    bool reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const override final;

    bool reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec3_t& reproj) const override final;

    nlohmann::json to_json() const override final;

    //! Override for optimization
    void undistort_points(const std::vector<cv::Point2f>& dist_pts, std::vector<cv::Point2f>& undist_pts) const override final;
    void undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypt, std::vector<cv::KeyPoint>& undist_keypt) const override final;

    //-------------------------
    // Parameters specific to this model

    //! pinhole params
    const double fx_;
    const double fy_;
    const double cx_;
    const double cy_;
    const double fx_inv_;
    const double fy_inv_;

    //! distortion params
    const double k1_;
    const double k2_;
    const double k3_;
    const double k4_;

    //! camera matrix in OpenCV format
    cv::Mat cv_cam_matrix_;
    //! camera matrix in Eigen format
    Mat33_t eigen_cam_matrix_;
    //! distortion params in OpenCV format
    cv::Mat cv_dist_params_;
    //! distortion params in Eigen format
    Vec4_t eigen_dist_params_;
};

std::ostream& operator<<(std::ostream& os, const fisheye& params);

} // namespace camera
} // namespace openvslam

#endif // OPENVSLAM_CAMERA_FISHEYE_H
