#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/bow_database.h"
#include "stella_vslam/module/relocalizer.h"
#include "stella_vslam/util/fancy_index.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace module {

relocalizer::relocalizer(const double bow_match_lowe_ratio, const double proj_match_lowe_ratio,
                         const double robust_match_lowe_ratio,
                         const unsigned int min_num_bow_matches, const unsigned int min_num_valid_obs)
    : min_num_bow_matches_(min_num_bow_matches), min_num_valid_obs_(min_num_valid_obs),
      bow_matcher_(bow_match_lowe_ratio, true), proj_matcher_(proj_match_lowe_ratio, true),
      robust_matcher_(robust_match_lowe_ratio, false),
      pose_optimizer_() {
    spdlog::debug("CONSTRUCT: module::relocalizer");
}

relocalizer::relocalizer(const YAML::Node& yaml_node)
    : relocalizer(yaml_node["bow_match_lowe_ratio"].as<double>(0.75),
                  yaml_node["proj_match_lowe_ratio"].as<double>(0.9),
                  yaml_node["robust_match_lowe_ratio"].as<double>(0.8),
                  yaml_node["min_num_bow_matches"].as<unsigned int>(20),
                  yaml_node["min_num_valid_obs"].as<unsigned int>(50)) {
}

relocalizer::~relocalizer() {
    spdlog::debug("DESTRUCT: module::relocalizer");
}

bool relocalizer::relocalize(data::bow_database* bow_db, data::frame& curr_frm) {
    // Acquire relocalization candidates
    const auto reloc_candidates = bow_db->acquire_relocalization_candidates(&curr_frm);
    if (reloc_candidates.empty()) {
        return false;
    }

    return reloc_by_candidates(curr_frm, reloc_candidates);
}

bool relocalizer::reloc_by_candidates(data::frame& curr_frm,
                                      const std::vector<std::shared_ptr<stella_vslam::data::keyframe>>& reloc_candidates,
                                      bool use_robust_matcher) {
    const auto num_candidates = reloc_candidates.size();

    spdlog::debug("Start relocalization. Number of candidate keyframes is {}", num_candidates);

    // Compute matching points for each candidate by using BoW tree matcher
    for (unsigned int i = 0; i < num_candidates; ++i) {
        const auto& candidate_keyfrm = reloc_candidates.at(i);
        if (candidate_keyfrm->will_be_erased()) {
            spdlog::debug("keyframe will be erased. candidate keyframe id is {}", candidate_keyfrm->id_);
            return false;
        }

        bool ok = reloc_by_candidate(curr_frm, candidate_keyfrm, use_robust_matcher);
        if (ok) {
            spdlog::info("relocalization succeeded");
            // TODO: should set the reference keyframe of the current frame
            return true;
        }
    }

    curr_frm.cam_pose_cw_is_valid_ = false;
    return false;
}

bool relocalizer::reloc_by_candidate(data::frame& curr_frm,
                                     const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                                     bool use_robust_matcher) {
    std::vector<unsigned int> inlier_indices;
    std::vector<std::shared_ptr<data::landmark>> matched_landmarks;
    bool ok = relocalize_by_pnp_solver(curr_frm, candidate_keyfrm, use_robust_matcher, inlier_indices, matched_landmarks);
    if (!ok) {
        return false;
    }

    // Set 2D-3D matches for the pose optimization
    curr_frm.landmarks_ = std::vector<std::shared_ptr<data::landmark>>(curr_frm.frm_obs_.num_keypts_, nullptr);
    for (const auto idx : inlier_indices) {
        // Set only the valid 3D points to the current frame
        curr_frm.landmarks_.at(idx) = matched_landmarks.at(idx);
    }

    std::vector<bool> outlier_flags;
    ok = optimize_pose(curr_frm, candidate_keyfrm, outlier_flags);
    if (!ok) {
        return false;
    }

    std::set<std::shared_ptr<data::landmark>> already_found_landmarks;
    for (const auto idx : inlier_indices) {
        if (outlier_flags.at(idx)) {
            continue;
        }
        // Record the 3D points already associated to the frame keypoints
        already_found_landmarks.insert(matched_landmarks.at(idx));
    }

    ok = refine_pose(curr_frm, candidate_keyfrm, already_found_landmarks);
    return ok;
}

bool relocalizer::relocalize_by_pnp_solver(data::frame& curr_frm,
                                           const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                                           bool use_robust_matcher,
                                           std::vector<unsigned int>& inlier_indices,
                                           std::vector<std::shared_ptr<data::landmark>>& matched_landmarks) const {
    const auto num_matches = use_robust_matcher ? robust_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, matched_landmarks)
                                                : bow_matcher_.match_frame_and_keyframe(candidate_keyfrm, curr_frm, matched_landmarks);
    // Discard the candidate if the number of 2D-3D matches is less than the threshold
    if (num_matches < min_num_bow_matches_) {
        spdlog::debug("Number of 2D-3D matches ({}) < threshold ({}). candidate keyframe id is {}", num_matches, min_num_bow_matches_, candidate_keyfrm->id_);
        return false;
    }

    // Setup an PnP solver with the current 2D-3D matches
    const auto valid_indices = extract_valid_indices(matched_landmarks);
    auto pnp_solver = setup_pnp_solver(valid_indices, curr_frm.frm_obs_.bearings_, curr_frm.frm_obs_.undist_keypts_,
                                       matched_landmarks, curr_frm.orb_params_->scale_factors_);

    // 1. Estimate the camera pose using EPnP (+ RANSAC)

    pnp_solver->find_via_ransac(30);
    if (!pnp_solver->solution_is_valid()) {
        spdlog::debug("solution is not valid. candidate keyframe id is {}", candidate_keyfrm->id_);
        return false;
    }

    curr_frm.cam_pose_cw_ = pnp_solver->get_best_cam_pose();
    curr_frm.update_pose_params();

    // Get the inlier indices after EPnP+RANSAC
    inlier_indices = util::resample_by_indices(valid_indices, pnp_solver->get_inlier_flags());

    return true;
}

bool relocalizer::optimize_pose(data::frame& curr_frm,
                                const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                                std::vector<bool>& outlier_flags) const {
    // Pose optimization
    g2o::SE3Quat optimized_pose;
    auto num_valid_obs = pose_optimizer_.optimize(curr_frm, optimized_pose, outlier_flags);
    curr_frm.set_cam_pose(optimized_pose);

    // Discard the candidate if the number of the inliers is less than the threshold
    if (num_valid_obs < min_num_bow_matches_ / 2) {
        spdlog::debug("Number of inliers ({}) < threshold ({}). candidate keyframe id is {}", num_valid_obs, min_num_bow_matches_ / 2, candidate_keyfrm->id_);
        return false;
    }

    // Reject outliers
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.num_keypts_; idx++) {
        if (!outlier_flags.at(idx)) {
            continue;
        }
        curr_frm.landmarks_.at(idx) = nullptr;
    }

    return true;
}

bool relocalizer::refine_pose(data::frame& curr_frm,
                              const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                              const std::set<std::shared_ptr<data::landmark>>& already_found_landmarks) const {
    // 3. Apply projection match to increase 2D-3D matches

    auto num_valid_obs = already_found_landmarks.size();

    // Projection match based on the pre-optimized camera pose
    auto num_found = proj_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, already_found_landmarks, 10, 100);
    // Discard the candidate if the number of the inliers is less than the threshold
    if (num_valid_obs + num_found < min_num_valid_obs_) {
        spdlog::debug("Number of inliers ({}) < threshold ({}). candidate keyframe id is {}", num_valid_obs + num_found, min_num_valid_obs_, candidate_keyfrm->id_);
        return false;
    }

    g2o::SE3Quat optimized_pose1;
    std::vector<bool> outlier_flags1;
    auto num_valid_obs1 = pose_optimizer_.optimize(curr_frm, optimized_pose1, outlier_flags1);
    curr_frm.set_cam_pose(optimized_pose1);

    // Exclude the already-associated landmarks
    std::set<std::shared_ptr<data::landmark>> already_found_landmarks1;
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.num_keypts_; ++idx) {
        if (!curr_frm.landmarks_.at(idx)) {
            continue;
        }
        already_found_landmarks1.insert(curr_frm.landmarks_.at(idx));
    }
    // Apply projection match again, then set the 2D-3D matches
    auto num_additional = proj_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, already_found_landmarks1, 3, 64);

    // Discard if the number of the observations is less than the threshold
    if (num_valid_obs1 + num_additional < min_num_valid_obs_) {
        spdlog::debug("Number of observations ({}) < threshold ({}). candidate keyframe id is {}", num_valid_obs1 + num_additional, min_num_valid_obs_, candidate_keyfrm->id_);
        return false;
    }

    // Perform optimization again
    g2o::SE3Quat optimized_pose2;
    std::vector<bool> outlier_flags2;
    auto num_valid_obs2 = pose_optimizer_.optimize(curr_frm, optimized_pose2, outlier_flags2);
    curr_frm.set_cam_pose(optimized_pose2);

    // Discard if falling below the threshold
    if (num_valid_obs2 < min_num_valid_obs_) {
        spdlog::debug("Number of observatoins ({}) < threshold ({}). candidate keyframe id is {}", num_valid_obs2, min_num_valid_obs_, candidate_keyfrm->id_);
        return false;
    }

    // Reject outliers
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.num_keypts_; ++idx) {
        if (!outlier_flags2.at(idx)) {
            continue;
        }
        curr_frm.landmarks_.at(idx) = nullptr;
    }

    return true;
}

std::vector<unsigned int> relocalizer::extract_valid_indices(const std::vector<std::shared_ptr<data::landmark>>& landmarks) const {
    std::vector<unsigned int> valid_indices;
    valid_indices.reserve(landmarks.size());
    for (unsigned int idx = 0; idx < landmarks.size(); ++idx) {
        auto lm = landmarks.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        valid_indices.push_back(idx);
    }
    return valid_indices;
}

std::unique_ptr<solve::pnp_solver> relocalizer::setup_pnp_solver(const std::vector<unsigned int>& valid_indices,
                                                                 const eigen_alloc_vector<Vec3_t>& bearings,
                                                                 const std::vector<cv::KeyPoint>& keypts,
                                                                 const std::vector<std::shared_ptr<data::landmark>>& matched_landmarks,
                                                                 const std::vector<float>& scale_factors) const {
    // Resample valid elements
    const auto valid_bearings = util::resample_by_indices(bearings, valid_indices);
    const auto valid_keypts = util::resample_by_indices(keypts, valid_indices);
    const auto valid_assoc_lms = util::resample_by_indices(matched_landmarks, valid_indices);
    eigen_alloc_vector<Vec3_t> valid_landmarks(valid_indices.size());
    for (unsigned int i = 0; i < valid_indices.size(); ++i) {
        valid_landmarks.at(i) = valid_assoc_lms.at(i)->get_pos_in_world();
    }
    // Setup PnP solver
    return std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(valid_bearings, valid_keypts, valid_landmarks, scale_factors));
}

} // namespace module
} // namespace stella_vslam
