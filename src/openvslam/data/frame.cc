#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"
#include "openvslam/camera/radial_division.h"
#include "openvslam/data/common.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/feature/orb_extractor.h"
#include "openvslam/match/stereo.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace data {

std::atomic<unsigned int> frame::next_id_{0};

frame::frame(const double timestamp, camera::base* camera, feature::orb_params* orb_params,
             const frame_observation frm_obs)
    : id_(next_id_++), timestamp_(timestamp), camera_(camera), orb_params_(orb_params), frm_obs_(frm_obs),
      // Initialize association with 3D points
      landmarks_(std::vector<std::shared_ptr<landmark>>(frm_obs_.num_keypts_, nullptr)),
      outlier_flags_(std::vector<bool>(frm_obs_.num_keypts_, false)) {}

void frame::set_cam_pose(const Mat44_t& cam_pose_cw) {
    cam_pose_cw_is_valid_ = true;
    cam_pose_cw_ = cam_pose_cw;
    update_pose_params();
}

void frame::set_cam_pose(const g2o::SE3Quat& cam_pose_cw) {
    set_cam_pose(util::converter::to_eigen_mat(cam_pose_cw));
}

Mat44_t frame::get_cam_pose() const {
    return cam_pose_cw_;
}

Mat44_t frame::get_cam_pose_inv() const {
    Mat44_t cam_pose_wc = Mat44_t::Identity();
    cam_pose_wc.block<3, 3>(0, 0) = rot_wc_;
    cam_pose_wc.block<3, 1>(0, 3) = cam_center_;
    return cam_pose_wc;
}

void frame::update_pose_params() {
    rot_cw_ = cam_pose_cw_.block<3, 3>(0, 0);
    rot_wc_ = rot_cw_.transpose();
    trans_cw_ = cam_pose_cw_.block<3, 1>(0, 3);
    cam_center_ = -rot_cw_.transpose() * trans_cw_;
}

Vec3_t frame::get_cam_center() const {
    return cam_center_;
}

Mat33_t frame::get_rotation_inv() const {
    return rot_wc_;
}

bool frame::bow_is_available() const {
    return !bow_vec_.empty() && !bow_feat_vec_.empty();
}

void frame::compute_bow(bow_vocabulary* bow_vocab) {
    bow_vocabulary_util::compute_bow(bow_vocab, frm_obs_.descriptors_, bow_vec_, bow_feat_vec_);
}

bool frame::can_observe(const std::shared_ptr<landmark>& lm, const float ray_cos_thr,
                        Vec2_t& reproj, float& x_right, unsigned int& pred_scale_level) const {
    const Vec3_t pos_w = lm->get_pos_in_world();

    const bool in_image = camera_->reproject_to_image(rot_cw_, trans_cw_, pos_w, reproj, x_right);
    if (!in_image) {
        return false;
    }

    const Vec3_t cam_to_lm_vec = pos_w - cam_center_;
    const auto cam_to_lm_dist = cam_to_lm_vec.norm();
    if (!lm->is_inside_in_orb_scale(cam_to_lm_dist)) {
        return false;
    }

    const Vec3_t obs_mean_normal = lm->get_obs_mean_normal();
    const auto ray_cos = cam_to_lm_vec.dot(obs_mean_normal) / cam_to_lm_dist;
    if (ray_cos < ray_cos_thr) {
        return false;
    }

    pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, this->orb_params_->num_levels_, this->orb_params_->log_scale_factor_);
    return true;
}

std::vector<unsigned int> frame::get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level, const int max_level) const {
    return data::get_keypoints_in_cell(camera_, frm_obs_.undist_keypts_, frm_obs_.keypt_indices_in_cells_, ref_x, ref_y, margin, min_level, max_level);
}

Vec3_t frame::triangulate_stereo(const unsigned int idx) const {
    assert(camera_->setup_type_ != camera::setup_type_t::Monocular);

    switch (camera_->model_type_) {
        case camera::model_type_t::Perspective: {
            auto camera = static_cast<camera::perspective*>(camera_);

            const float depth = frm_obs_.depths_.at(idx);
            if (0.0 < depth) {
                const float x = frm_obs_.undist_keypts_.at(idx).pt.x;
                const float y = frm_obs_.undist_keypts_.at(idx).pt.y;
                const float unproj_x = (x - camera->cx_) * depth * camera->fx_inv_;
                const float unproj_y = (y - camera->cy_) * depth * camera->fy_inv_;
                const Vec3_t pos_c{unproj_x, unproj_y, depth};

                // Convert from camera coordinates to world coordinates
                return rot_wc_ * pos_c + cam_center_;
            }
            else {
                return Vec3_t::Zero();
            }
        }
        case camera::model_type_t::Fisheye: {
            auto camera = static_cast<camera::fisheye*>(camera_);

            const float depth = frm_obs_.depths_.at(idx);
            if (0.0 < depth) {
                const float x = frm_obs_.undist_keypts_.at(idx).pt.x;
                const float y = frm_obs_.undist_keypts_.at(idx).pt.y;
                const float unproj_x = (x - camera->cx_) * depth * camera->fx_inv_;
                const float unproj_y = (y - camera->cy_) * depth * camera->fy_inv_;
                const Vec3_t pos_c{unproj_x, unproj_y, depth};

                // Convert from camera coordinates to world coordinates
                return rot_wc_ * pos_c + cam_center_;
            }
            else {
                return Vec3_t::Zero();
            }
        }
        case camera::model_type_t::Equirectangular: {
            throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
        }
        case camera::model_type_t::RadialDivision: {
            auto camera = static_cast<camera::radial_division*>(camera_);

            const float depth = frm_obs_.depths_.at(idx);
            if (0.0 < depth) {
                const float x = frm_obs_.keypts_.at(idx).pt.x;
                const float y = frm_obs_.keypts_.at(idx).pt.y;
                const float unproj_x = (x - camera->cx_) * depth * camera->fx_inv_;
                const float unproj_y = (y - camera->cy_) * depth * camera->fy_inv_;
                const Vec3_t pos_c{unproj_x, unproj_y, depth};

                // camera座標 -> world座標
                return rot_wc_ * pos_c + cam_center_;
            }
            else {
                return Vec3_t::Zero();
            }
        }
    }

    return Vec3_t::Zero();
}

} // namespace data
} // namespace openvslam
