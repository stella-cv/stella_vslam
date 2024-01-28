#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/bow_database.h"
#include "stella_vslam/module/local_map_updater.h"
#include "stella_vslam/module/relocalizer.h"
#include "stella_vslam/optimize/pose_optimizer_g2o.h"
#include "stella_vslam/util/fancy_index.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace module {

relocalizer::relocalizer(const std::shared_ptr<optimize::pose_optimizer>& pose_optimizer,
                         const double bow_match_lowe_ratio, const double proj_match_lowe_ratio,
                         const double robust_match_lowe_ratio,
                         const unsigned int min_num_bow_matches, const unsigned int min_num_valid_obs,
                         const bool use_fixed_seed,
                         const bool search_neighbor,
                         const unsigned int top_n_covisibilities_to_search,
                         const float num_common_words_thr_ratio,
                         const unsigned int max_num_ransac_iter,
                         const unsigned int max_num_local_keyfrms)
    : min_num_bow_matches_(min_num_bow_matches), min_num_valid_obs_(min_num_valid_obs),
      bow_matcher_(bow_match_lowe_ratio, false), proj_matcher_(proj_match_lowe_ratio, false),
      robust_matcher_(robust_match_lowe_ratio, false),
      pose_optimizer_(pose_optimizer), use_fixed_seed_(use_fixed_seed),
      search_neighbor_(search_neighbor),
      top_n_covisibilities_to_search_(top_n_covisibilities_to_search),
      num_common_words_thr_ratio_(num_common_words_thr_ratio),
      max_num_ransac_iter_(max_num_ransac_iter),
      max_num_local_keyfrms_(max_num_local_keyfrms) {
    spdlog::debug("CONSTRUCT: module::relocalizer");
}

relocalizer::relocalizer(const std::shared_ptr<optimize::pose_optimizer>& pose_optimizer, const YAML::Node& yaml_node)
    : relocalizer(pose_optimizer,
                  yaml_node["bow_match_lowe_ratio"].as<double>(0.75),
                  yaml_node["proj_match_lowe_ratio"].as<double>(0.9),
                  yaml_node["robust_match_lowe_ratio"].as<double>(0.8),
                  yaml_node["min_num_bow_matches"].as<unsigned int>(20),
                  yaml_node["min_num_valid_obs"].as<unsigned int>(50),
                  yaml_node["use_fixed_seed"].as<bool>(false),
                  yaml_node["search_neighbor"].as<bool>(true),
                  yaml_node["top_n_covisibilities_to_search"].as<unsigned int>(10),
                  yaml_node["num_common_words_thr_ratio"].as<float>(0.8f),
                  yaml_node["max_num_ransac_iter"].as<unsigned int>(30),
                  yaml_node["max_num_local_keyfrms"].as<unsigned int>(60)) {
}

relocalizer::~relocalizer() {
    spdlog::debug("DESTRUCT: module::relocalizer");
}

bool relocalizer::relocalize(data::bow_database* bow_db, data::frame& curr_frm) {
    // Acquire relocalization candidates
    const auto reloc_candidates = bow_db->acquire_keyframes(curr_frm.bow_vec_, 0.0f, num_common_words_thr_ratio_);
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
            continue;
        }

        bool ok = reloc_by_candidate(curr_frm, candidate_keyfrm, use_robust_matcher);
        if (ok) {
            spdlog::info("relocalization succeeded (frame={}, keyframe={})", curr_frm.id_, candidate_keyfrm->id_);
            // TODO: should set the reference keyframe of the current frame
            return true;
        }
    }

    curr_frm.invalidate_pose();
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
    curr_frm.erase_landmarks();
    for (const auto idx : inlier_indices) {
        // Set only the valid 3D points to the current frame
        curr_frm.add_landmark(matched_landmarks.at(idx), idx);
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
    if (!ok) {
        return false;
    }

    ok = refine_pose_by_local_map(curr_frm, candidate_keyfrm);
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

    if (search_neighbor_) {
        // Search additional association from neighbor keyframes
        auto ngh_keyfrms = candidate_keyfrm->graph_node_->get_top_n_covisibilities(top_n_covisibilities_to_search_);
        std::unordered_set<unsigned int> already_found_landmark_ids;
        for (const auto& lm : matched_landmarks) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }
            already_found_landmark_ids.insert(lm->id_);
        }

        for (const auto& ngh_keyfrm : ngh_keyfrms) {
            std::vector<std::shared_ptr<data::landmark>> additional_matched_landmarks;
            const auto num_additional_matches = use_robust_matcher ? robust_matcher_.match_frame_and_keyframe(curr_frm, ngh_keyfrm, additional_matched_landmarks)
                                                                   : bow_matcher_.match_frame_and_keyframe(ngh_keyfrm, curr_frm, additional_matched_landmarks);
            assert(matched_landmarks.size() == additional_matched_landmarks.size());
            int num_associated = 0;
            for (unsigned int idx = 0; idx < matched_landmarks.size(); ++idx) {
                auto lm = additional_matched_landmarks.at(idx);
                if (!lm) {
                    continue;
                }
                if (lm->will_be_erased()) {
                    continue;
                }
                if (already_found_landmark_ids.count(lm->id_)) {
                    continue;
                }
                // Add new association only if there is no matching landmark
                if (matched_landmarks[idx] != nullptr) {
                    continue;
                }
                matched_landmarks[idx] = lm;
                already_found_landmark_ids.insert(lm->id_);
                num_associated++;
            }
            SPDLOG_TRACE("Number of additional 2D-3D matches (detected={}, associated={}). neighbor keyframe={}", num_additional_matches, num_associated, ngh_keyfrm->id_);
        }
    }

    // Setup an PnP solver with the current 2D-3D matches
    const auto valid_indices = extract_valid_indices(matched_landmarks);
    auto pnp_solver = setup_pnp_solver(valid_indices, curr_frm.frm_obs_.bearings_, curr_frm.frm_obs_.undist_keypts_,
                                       matched_landmarks, curr_frm.orb_params_->scale_factors_);

    // 1. Estimate the camera pose using EPnP (+ RANSAC)

    pnp_solver->find_via_ransac(max_num_ransac_iter_, false);
    if (!pnp_solver->solution_is_valid()) {
        spdlog::debug("solution is not valid. candidate keyframe id is {}", candidate_keyfrm->id_);
        return false;
    }

    curr_frm.set_pose_cw(pnp_solver->get_best_cam_pose());

    // Get the inlier indices after EPnP+RANSAC
    inlier_indices = util::resample_by_indices(valid_indices, pnp_solver->get_inlier_flags());

    return true;
}

bool relocalizer::optimize_pose(data::frame& curr_frm,
                                const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                                std::vector<bool>& outlier_flags) const {
    // Pose optimization
    Mat44_t optimized_pose;
    auto num_valid_obs = pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
    curr_frm.set_pose_cw(optimized_pose);

    // Discard the candidate if the number of the inliers is less than the threshold
    if (num_valid_obs < min_num_bow_matches_ / 2) {
        spdlog::debug("Number of inliers ({}) < threshold ({}). candidate keyframe id is {}", num_valid_obs, min_num_bow_matches_ / 2, candidate_keyfrm->id_);
        return false;
    }

    // Reject outliers
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); idx++) {
        if (!outlier_flags.at(idx)) {
            continue;
        }
        curr_frm.erase_landmark_with_index(idx);
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

    Mat44_t optimized_pose1;
    std::vector<bool> outlier_flags1;
    auto num_valid_obs1 = pose_optimizer_->optimize(curr_frm, optimized_pose1, outlier_flags1);
    SPDLOG_TRACE("refine_pose: num_valid_obs1={}", num_valid_obs1);
    curr_frm.set_pose_cw(optimized_pose1);

    // Exclude the already-associated landmarks
    std::set<std::shared_ptr<data::landmark>> already_found_landmarks1;
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
        const auto& lm = curr_frm.get_landmark(idx);
        if (!lm) {
            continue;
        }
        already_found_landmarks1.insert(lm);
    }
    // Apply projection match again, then set the 2D-3D matches
    auto num_additional = proj_matcher_.match_frame_and_keyframe(curr_frm, candidate_keyfrm, already_found_landmarks1, 3, 64);

    // Discard if the number of the observations is less than the threshold
    if (num_valid_obs1 + num_additional < min_num_valid_obs_) {
        spdlog::debug("Number of observations ({}) < threshold ({}). candidate keyframe id is {}", num_valid_obs1 + num_additional, min_num_valid_obs_, candidate_keyfrm->id_);
        return false;
    }

    // Perform optimization again
    Mat44_t optimized_pose2;
    std::vector<bool> outlier_flags2;
    auto num_valid_obs2 = pose_optimizer_->optimize(curr_frm, optimized_pose2, outlier_flags2);
    SPDLOG_TRACE("refine_pose: num_valid_obs2={}", num_valid_obs2);
    curr_frm.set_pose_cw(optimized_pose2);

    // Discard if falling below the threshold
    if (num_valid_obs2 < min_num_valid_obs_) {
        spdlog::debug("Number of observatoins ({}) < threshold ({}). candidate keyframe id is {}", num_valid_obs2, min_num_valid_obs_, candidate_keyfrm->id_);
        return false;
    }

    // Reject outliers
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
        if (!outlier_flags2.at(idx)) {
            continue;
        }
        curr_frm.erase_landmark_with_index(idx);
    }

    return true;
}

bool relocalizer::refine_pose_by_local_map(data::frame& curr_frm,
                                           const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm) const {
    // Create local map
    auto local_map_updater = module::local_map_updater(max_num_local_keyfrms_);
    if (!local_map_updater.acquire_local_map(curr_frm.get_landmarks())) {
        return false;
    }
    auto local_keyfrms = local_map_updater.get_local_keyframes();
    auto local_landmarks = local_map_updater.get_local_landmarks();
    auto nearest_covisibility = local_map_updater.get_nearest_covisibility();
    SPDLOG_TRACE("refine_pose_by_local_map: keyfrms={}, lms={} nearest_covisibility id={}", local_keyfrms.size(), local_landmarks.size(), nearest_covisibility->id_);

    std::vector<int> margins{5, 15, 5};
    for (size_t i = 0; i < margins.size(); ++i) {
        // select the landmarks which can be reprojected from the ones observed in the current frame
        std::unordered_set<unsigned int> curr_landmark_ids;
        for (const auto& lm : curr_frm.get_landmarks()) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            // this landmark cannot be reprojected
            // because already observed in the current frame
            curr_landmark_ids.insert(lm->id_);
        }

        bool found_proj_candidate = false;
        // temporary variables
        Vec2_t reproj;
        float x_right;
        unsigned int pred_scale_level;
        eigen_alloc_unord_map<unsigned int, Vec2_t> lm_to_reproj;
        std::unordered_map<unsigned int, float> lm_to_x_right;
        std::unordered_map<unsigned int, unsigned int> lm_to_scale;
        for (const auto& lm : local_landmarks) {
            if (curr_landmark_ids.count(lm->id_)) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            // check the observability
            if (curr_frm.can_observe(lm, 0.5, reproj, x_right, pred_scale_level)) {
                lm_to_reproj[lm->id_] = reproj;
                lm_to_x_right[lm->id_] = x_right;
                lm_to_scale[lm->id_] = pred_scale_level;

                found_proj_candidate = true;
            }
        }

        if (!found_proj_candidate) {
            return false;
        }

        // acquire more 2D-3D matches by projecting the local landmarks to the current frame
        match::projection projection_matcher(0.8);
        const float margin = margins[i];
        auto num_additional_matches = projection_matcher.match_frame_and_landmarks(curr_frm, local_landmarks, lm_to_reproj, lm_to_x_right, lm_to_scale, margin);

        // optimize the pose
        Mat44_t optimized_pose;
        std::vector<bool> outlier_flags;
        auto num_valid_obs = pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
        curr_frm.set_pose_cw(optimized_pose);

        // Reject outliers
        for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
            if (!outlier_flags.at(idx)) {
                continue;
            }
            curr_frm.erase_landmark_with_index(idx);
        }
        SPDLOG_TRACE("refine_pose_by_local_map: iter={:2}, margin={:2}, num_additional_matches={:4}, num_valid_obs={:4}", i, margin, num_additional_matches, num_valid_obs);

        if (i == margins.size() - 1) {
            const auto num_tracked_lms = candidate_keyfrm->get_num_tracked_landmarks(0);
            const double ratio = 0.2;
            SPDLOG_TRACE("refine_pose_by_local_map: num_valid_obs={:4}, num_tracked_lms={:4}", num_valid_obs, num_tracked_lms);
            if (num_valid_obs < num_tracked_lms * ratio) {
                return false;
            }
        }
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
    std::vector<int> octaves(valid_indices.size());
    for (unsigned int i = 0; i < valid_indices.size(); ++i) {
        octaves.at(i) = valid_keypts.at(i).octave;
    }
    const auto valid_assoc_lms = util::resample_by_indices(matched_landmarks, valid_indices);
    eigen_alloc_vector<Vec3_t> valid_points(valid_indices.size());
    for (unsigned int i = 0; i < valid_indices.size(); ++i) {
        valid_points.at(i) = valid_assoc_lms.at(i)->get_pos_in_world();
    }
    // Setup PnP solver
    return std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(valid_bearings, octaves, valid_points, scale_factors, 10, use_fixed_seed_));
}

} // namespace module
} // namespace stella_vslam
