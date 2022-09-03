#include "stella_vslam/camera/base.h"
#include "stella_vslam/data/bow_vocabulary.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/frame_observation.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/match/robust.h"
#include "stella_vslam/solve/essential_solver.h"
#include "stella_vslam/util/angle.h"

namespace stella_vslam {
namespace match {

unsigned int robust::match_for_triangulation(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2, const Mat33_t& E_12,
                                             std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs) const {
    unsigned int num_matches = 0;

    // Project the center of keyframe 1 to keyframe 2
    // to acquire the epipole coordinates of the candidate keyframe
    const Vec3_t cam_center_1 = keyfrm_1->get_trans_wc();
    const Mat33_t rot_2w = keyfrm_2->get_rot_cw();
    const Vec3_t trans_2w = keyfrm_2->get_trans_cw();
    Vec3_t epiplane_in_keyfrm_2;
    const bool valid_epiplane = keyfrm_2->camera_->reproject_to_bearing(rot_2w, trans_2w, cam_center_1, epiplane_in_keyfrm_2);

    // Acquire the 3D point information of the keframes
    const auto assoc_lms_in_keyfrm_1 = keyfrm_1->get_landmarks();
    const auto assoc_lms_in_keyfrm_2 = keyfrm_2->get_landmarks();

    // Save the matching information
    // Discard the already matched keypoints in keyframe 2
    // to acquire a unique association to each keypoint in keyframe 1
    std::vector<bool> is_already_matched_in_keyfrm_2(keyfrm_2->frm_obs_.num_keypts_, false);
    // Save the keypoint idx in keyframe 2 which is already associated to the keypoint idx in keyframe 1
    std::vector<int> matched_indices_2_in_keyfrm_1(keyfrm_1->frm_obs_.num_keypts_, -1);

    data::bow_feature_vector::const_iterator itr_1 = keyfrm_1->bow_feat_vec_.begin();
    data::bow_feature_vector::const_iterator itr_2 = keyfrm_2->bow_feat_vec_.begin();
    const data::bow_feature_vector::const_iterator itr_1_end = keyfrm_1->bow_feat_vec_.end();
    const data::bow_feature_vector::const_iterator itr_2_end = keyfrm_2->bow_feat_vec_.end();

    while (itr_1 != itr_1_end && itr_2 != itr_2_end) {
        // Check if the node numbers of BoW tree match
        if (itr_1->first == itr_2->first) {
            // If the node numbers of BoW tree match,
            // Check in practice if matches exist between keyframes
            const auto& keyfrm_1_indices = itr_1->second;
            const auto& keyfrm_2_indices = itr_2->second;

            for (const auto idx_1 : keyfrm_1_indices) {
                const auto& lm_1 = assoc_lms_in_keyfrm_1.at(idx_1);
                // 3次元点が存在"する"場合はスルー(triangulation前のmatchingであるため)
                if (lm_1) {
                    continue;
                }

                // Check if it's a stereo keypoint or not
                const bool is_stereo_keypt_1 = !keyfrm_1->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm_1->frm_obs_.stereo_x_right_.at(idx_1);

                // Acquire the keypoints and ORB feature vectors
                const auto& keypt_1 = keyfrm_1->frm_obs_.undist_keypts_.at(idx_1);
                const Vec3_t& bearing_1 = keyfrm_1->frm_obs_.bearings_.at(idx_1);
                const auto& desc_1 = keyfrm_1->frm_obs_.descriptors_.row(idx_1);

                // Find a keypoint in keyframe 2 that has the minimum hamming distance
                unsigned int best_hamm_dist = HAMMING_DIST_THR_LOW;
                int best_idx_2 = -1;

                for (const auto idx_2 : keyfrm_2_indices) {
                    // Ignore if the keypoint is associated any 3D points
                    // (because this function is used for triangulation)
                    const auto& lm_2 = assoc_lms_in_keyfrm_2.at(idx_2);
                    if (lm_2) {
                        continue;
                    }

                    // Ignore if matches are already aquired
                    if (is_already_matched_in_keyfrm_2.at(idx_2)) {
                        continue;
                    }

                    if (check_orientation_ && std::abs(util::angle::diff(keypt_1.angle, keyfrm_2->frm_obs_.undist_keypts_.at(idx_2).angle)) > 30.0) {
                        continue;
                    }

                    // Check if it's a stereo keypoint or not
                    const bool is_stereo_keypt_2 = !keyfrm_2->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm_2->frm_obs_.stereo_x_right_.at(idx_2);

                    // Acquire the keypoints and ORB feature vectors
                    const Vec3_t& bearing_2 = keyfrm_2->frm_obs_.bearings_.at(idx_2);
                    const auto& desc_2 = keyfrm_2->frm_obs_.descriptors_.row(idx_2);

                    // Compute the distance
                    const auto hamm_dist = compute_descriptor_distance_32(desc_1, desc_2);

                    if (HAMMING_DIST_THR_LOW < hamm_dist || best_hamm_dist < hamm_dist) {
                        continue;
                    }

                    if (valid_epiplane && !is_stereo_keypt_1 && !is_stereo_keypt_2) {
                        // Do not use any keypoints near the epipole if both are not stereo keypoints
                        const auto cos_dist = epiplane_in_keyfrm_2.dot(bearing_2);
                        // The threshold of the minimum angle formed by the epipole and the bearing vector is 3.0 degree
                        constexpr double cos_dist_thr = 0.99862953475;

                        // Do not allow to match if the formed angle is narrower that the threshold value
                        if (cos_dist_thr < cos_dist) {
                            continue;
                        }
                    }

                    // Check consistency in Matrix E
                    const bool is_inlier = check_epipolar_constraint(bearing_1, bearing_2, E_12,
                                                                     keyfrm_1->orb_params_->scale_factors_.at(keypt_1.octave));
                    if (is_inlier) {
                        best_idx_2 = idx_2;
                        best_hamm_dist = hamm_dist;
                    }
                }

                if (best_idx_2 < 0) {
                    continue;
                }

                is_already_matched_in_keyfrm_2.at(best_idx_2) = true;
                matched_indices_2_in_keyfrm_1.at(idx_1) = best_idx_2;
                ++num_matches;
            }

            ++itr_1;
            ++itr_2;
        }
        else if (itr_1->first < itr_2->first) {
            // Since the node number of keyframe 1 is smaller, increment the iterator until the node numbers match
            itr_1 = keyfrm_1->bow_feat_vec_.lower_bound(itr_2->first);
        }
        else {
            // Since the node number of keyframe 2 is smaller, increment the iterator until the node numbers match
            itr_2 = keyfrm_2->bow_feat_vec_.lower_bound(itr_1->first);
        }
    }

    matched_idx_pairs.clear();
    matched_idx_pairs.reserve(num_matches);

    for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_keyfrm_1.size(); ++idx_1) {
        if (matched_indices_2_in_keyfrm_1.at(idx_1) < 0) {
            continue;
        }
        matched_idx_pairs.emplace_back(std::make_pair(idx_1, matched_indices_2_in_keyfrm_1.at(idx_1)));
    }

    return num_matches;
}

unsigned int robust::match_keyframes(const std::shared_ptr<data::keyframe>& keyfrm1, const std::shared_ptr<data::keyframe>& keyfrm2,
                                     std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm,
                                     bool validate_with_essential_solver, bool use_fixed_seed) const {
    // Initialization
    const auto num_frm_keypts = keyfrm1->frm_obs_.num_keypts_;
    const auto keyfrm_lms = keyfrm2->get_landmarks();
    unsigned int num_inlier_matches = 0;
    matched_lms_in_frm = std::vector<std::shared_ptr<data::landmark>>(num_frm_keypts, nullptr);

    // Compute brute-force match
    std::vector<std::pair<int, int>> matches;
    brute_force_match(keyfrm1->frm_obs_, keyfrm2, matches);

    // Extract only inliers with eight-point RANSAC
    if (validate_with_essential_solver) {
        solve::essential_solver solver(keyfrm1->frm_obs_.bearings_, keyfrm2->frm_obs_.bearings_, matches, use_fixed_seed);
        solver.find_via_ransac(50, false);
        if (!solver.solution_is_valid()) {
            return 0;
        }
        const auto is_inlier_matches = solver.get_inlier_matches();

        // Save the information
        for (unsigned int i = 0; i < matches.size(); ++i) {
            if (!is_inlier_matches.at(i)) {
                continue;
            }
            const auto frm_idx = matches.at(i).first;
            const auto keyfrm_idx = matches.at(i).second;

            matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
            ++num_inlier_matches;
        }
    }
    else {
        // Save the information
        for (unsigned int i = 0; i < matches.size(); ++i) {
            const auto frm_idx = matches.at(i).first;
            const auto keyfrm_idx = matches.at(i).second;

            matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
            ++num_inlier_matches;
        }
    }

    return num_inlier_matches;
}

unsigned int robust::match_frame_and_keyframe(data::frame& frm, const std::shared_ptr<data::keyframe>& keyfrm,
                                              std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm,
                                              bool use_fixed_seed) const {
    // Initialization
    const auto num_frm_keypts = frm.frm_obs_.num_keypts_;
    const auto keyfrm_lms = keyfrm->get_landmarks();
    unsigned int num_inlier_matches = 0;
    matched_lms_in_frm = std::vector<std::shared_ptr<data::landmark>>(num_frm_keypts, nullptr);

    // Compute brute-force match
    std::vector<std::pair<int, int>> matches;
    brute_force_match(frm.frm_obs_, keyfrm, matches);

    // Extract only inliers with eight-point RANSAC
    solve::essential_solver solver(frm.frm_obs_.bearings_, keyfrm->frm_obs_.bearings_, matches, use_fixed_seed);
    solver.find_via_ransac(50, false);
    if (!solver.solution_is_valid()) {
        return 0;
    }
    const auto is_inlier_matches = solver.get_inlier_matches();

    // Save the information
    for (unsigned int i = 0; i < matches.size(); ++i) {
        if (!is_inlier_matches.at(i)) {
            continue;
        }
        const auto frm_idx = matches.at(i).first;
        const auto keyfrm_idx = matches.at(i).second;

        matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
        ++num_inlier_matches;
    }

    return num_inlier_matches;
}

unsigned int robust::brute_force_match(const data::frame_observation& frm_obs, const std::shared_ptr<data::keyframe>& keyfrm, std::vector<std::pair<int, int>>& matches) const {
    unsigned int num_matches = 0;

    // 1. Acquire the frame and keyframe information

    const auto num_keypts_1 = frm_obs.num_keypts_;
    const auto num_keypts_2 = keyfrm->frm_obs_.num_keypts_;
    const auto keypts_1 = frm_obs.undist_keypts_;
    const auto keypts_2 = keyfrm->frm_obs_.undist_keypts_;
    const auto lms_2 = keyfrm->get_landmarks();
    const auto& descs_1 = frm_obs.descriptors_;
    const auto& descs_2 = keyfrm->frm_obs_.descriptors_;

    // 2. Acquire ORB descriptors in the keyframe which are the first and second closest to the descriptors in the frame
    //    it is assumed that keypoint in the keyframe are associated to 3D points

    // Index 2 associated to each index 1
    auto matched_indices_2_in_1 = std::vector<int>(num_keypts_1, -1);
    // Avoid duplication
    std::unordered_set<int> already_matched_indices_1;

    for (unsigned int idx_2 = 0; idx_2 < num_keypts_2; ++idx_2) {
        // 3次元点が有効なもののみ対象にする
        const auto& lm_2 = lms_2.at(idx_2);
        if (!lm_2) {
            continue;
        }
        if (lm_2->will_be_erased()) {
            continue;
        }

        // Acquire the descriptor for index 2
        const auto& desc_2 = descs_2.row(idx_2);

        // Acquire the descriptors in the frame which are the first and second closest to the descriptor in the keyframe
        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx_1 = -1;
        unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;

        for (unsigned int idx_1 = 0; idx_1 < num_keypts_1; ++idx_1) {
            // Avoid duplication
            if (static_cast<bool>(already_matched_indices_1.count(idx_1))) {
                continue;
            }

            if (check_orientation_ && std::abs(util::angle::diff(keypts_1.at(idx_1).angle, keypts_2.at(idx_2).angle)) > 30.0) {
                continue;
            }

            const auto& desc_1 = descs_1.row(idx_1);

            const auto hamm_dist = compute_descriptor_distance_32(desc_2, desc_1);

            if (hamm_dist < best_hamm_dist) {
                second_best_hamm_dist = best_hamm_dist;
                best_hamm_dist = hamm_dist;
                best_idx_1 = idx_1;
            }
            else if (hamm_dist < second_best_hamm_dist) {
                second_best_hamm_dist = hamm_dist;
            }
        }

        if (HAMMING_DIST_THR_LOW < best_hamm_dist) {
            continue;
        }

        if (best_idx_1 < 0) {
            continue;
        }

        // Ratio test
        if (lowe_ratio_ * second_best_hamm_dist < static_cast<float>(best_hamm_dist)) {
            continue;
        }

        matched_indices_2_in_1.at(best_idx_1) = idx_2;
        // Avoid duplication
        already_matched_indices_1.insert(best_idx_1);

        ++num_matches;
    }

    matches.clear();
    matches.reserve(num_matches);
    for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_1.size(); ++idx_1) {
        const auto idx_2 = matched_indices_2_in_1.at(idx_1);
        if (idx_2 < 0) {
            continue;
        }
        matches.emplace_back(std::make_pair(idx_1, idx_2));
    }

    return num_matches;
}

bool robust::check_epipolar_constraint(const Vec3_t& bearing_1, const Vec3_t& bearing_2,
                                       const Mat33_t& E_12, const float bearing_1_scale_factor) const {
    // Normal vector of the epipolar plane on keyframe 1
    const Vec3_t epiplane_in_1 = E_12 * bearing_2;

    // Acquire the angle formed by the normal vector and the bearing
    const auto cos_residual = epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm();
    const auto residual_rad = M_PI / 2.0 - std::abs(std::acos(cos_residual));

    // The Inlier threshold value is 0.2 degree
    // (e.g. for the camera with width of 900-pixel and 90-degree FOV, 0.2 degree is equivalent to 2 pixel in the horizontal direction)
    // TODO: should prameterize the threshold
    constexpr double residual_deg_thr = 0.2;
    constexpr double residual_rad_thr = residual_deg_thr * M_PI / 180.0;

    // The larger keypoint scale permits less constraints
    // TODO: should consider the threshold weighting
    return residual_rad < residual_rad_thr * bearing_1_scale_factor;
}

} // namespace match
} // namespace stella_vslam
