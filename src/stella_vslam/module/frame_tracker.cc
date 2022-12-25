#include "stella_vslam/camera/base.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/match/bow_tree.h"
#include "stella_vslam/match/projection.h"
#include "stella_vslam/match/robust.h"
#include "stella_vslam/module/frame_tracker.h"
#include "stella_vslam/optimize/pose_optimizer_g2o.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace module {

frame_tracker::frame_tracker(camera::base* camera, const std::shared_ptr<optimize::pose_optimizer>& pose_optimizer,
                             const unsigned int num_matches_thr, bool use_fixed_seed)
    : camera_(camera), num_matches_thr_(num_matches_thr), use_fixed_seed_(use_fixed_seed), pose_optimizer_(pose_optimizer) {}

bool frame_tracker::motion_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity) const {
    match::projection projection_matcher(0.9, true);

    // Set the initial pose by using the motion model
    curr_frm.set_pose_cw(velocity * last_frm.get_pose_cw());

    // Initialize the 2D-3D matches
    curr_frm.erase_landmarks();

    // Reproject the 3D points observed in the last frame and find 2D-3D matches
    const float margin = (camera_->setup_type_ != camera::setup_type_t::Stereo) ? 20 : 10;
    auto num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, margin);

    if (num_matches < num_matches_thr_) {
        // Increment the margin, and search again
        curr_frm.erase_landmarks();
        num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, 2 * margin);
    }

    if (num_matches < num_matches_thr_) {
        spdlog::debug("motion based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // Pose optimization
    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
    curr_frm.set_pose_cw(optimized_pose);

    // Discard the outliers
    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        spdlog::debug("motion based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }
    else {
        return true;
    }
}

bool frame_tracker::bow_match_based_track(data::frame& curr_frm, const data::frame& last_frm, const std::shared_ptr<data::keyframe>& ref_keyfrm) const {
    match::bow_tree bow_matcher(0.7, true);

    // Search 2D-2D matches between the ref keyframes and the current frame
    // to acquire 2D-3D matches between the frame keypoints and 3D points observed in the ref keyframe
    std::vector<std::shared_ptr<data::landmark>> matched_lms_in_curr;
    auto num_matches = bow_matcher.match_frame_and_keyframe(ref_keyfrm, curr_frm, matched_lms_in_curr);

    if (num_matches < num_matches_thr_) {
        spdlog::debug("bow match based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // Update the 2D-3D matches
    curr_frm.set_landmarks(matched_lms_in_curr);

    // Pose optimization
    // The initial value is the pose of the previous frame
    curr_frm.set_pose_cw(last_frm.get_pose_cw());
    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
    curr_frm.set_pose_cw(optimized_pose);

    // Discard the outliers
    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        spdlog::debug("bow match based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }
    else {
        return true;
    }
}

bool frame_tracker::robust_match_based_track(data::frame& curr_frm, const data::frame& last_frm, const std::shared_ptr<data::keyframe>& ref_keyfrm) const {
    match::robust robust_matcher(0.8, true);

    // Search 2D-2D matches between the ref keyframes and the current frame
    // to acquire 2D-3D matches between the frame keypoints and 3D points observed in the ref keyframe
    std::vector<std::shared_ptr<data::landmark>> matched_lms_in_curr;
    auto num_matches = robust_matcher.match_frame_and_keyframe(curr_frm, ref_keyfrm, matched_lms_in_curr, use_fixed_seed_);

    if (num_matches < num_matches_thr_) {
        spdlog::debug("robust match based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // Update the 2D-3D matches
    curr_frm.set_landmarks(matched_lms_in_curr);

    // Pose optimization
    // The initial value is the pose of the previous frame
    curr_frm.set_pose_cw(last_frm.get_pose_cw());
    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
    curr_frm.set_pose_cw(optimized_pose);

    // Discard the outliers
    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        spdlog::debug("robust match based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }
    else {
        return true;
    }
}

unsigned int frame_tracker::discard_outliers(const std::vector<bool>& outlier_flags, data::frame& curr_frm) const {
    unsigned int num_valid_matches = 0;

    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.num_keypts_; ++idx) {
        if (curr_frm.get_landmark(idx) == nullptr) {
            continue;
        }

        if (outlier_flags.at(idx)) {
            curr_frm.erase_landmark_with_index(idx);
        }
        else {
            ++num_valid_matches;
        }
    }

    return num_valid_matches;
}

} // namespace module
} // namespace stella_vslam
