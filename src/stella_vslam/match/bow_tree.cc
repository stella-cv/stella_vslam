#include "stella_vslam/data/bow_vocabulary.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/match/bow_tree.h"
#include "stella_vslam/util/angle.h"

namespace stella_vslam {
namespace match {

unsigned int bow_tree::match_frame_and_keyframe(const std::shared_ptr<data::keyframe>& keyfrm, data::frame& frm, std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm) const {
    unsigned int num_matches = 0;

    matched_lms_in_frm = std::vector<std::shared_ptr<data::landmark>>(frm.frm_obs_.num_keypts_, nullptr);

    const auto keyfrm_lms = keyfrm->get_landmarks();

    data::bow_feature_vector::const_iterator keyfrm_itr = keyfrm->bow_feat_vec_.begin();
    data::bow_feature_vector::const_iterator frm_itr = frm.bow_feat_vec_.begin();
    const data::bow_feature_vector::const_iterator kryfrm_end = keyfrm->bow_feat_vec_.end();
    const data::bow_feature_vector::const_iterator frm_end = frm.bow_feat_vec_.end();

    while (keyfrm_itr != kryfrm_end && frm_itr != frm_end) {
        // Check if the node numbers of BoW tree match
        if (keyfrm_itr->first == frm_itr->first) {
            // If the node numbers of BoW tree match,
            // Check in practice if matches exist between the frame and keyframe
            const auto& keyfrm_indices = keyfrm_itr->second;
            const auto& frm_indices = frm_itr->second;

            for (const auto keyfrm_idx : keyfrm_indices) {
                // Ignore if the keypoint of keyframe is not associated any 3D points
                auto& lm = keyfrm_lms.at(keyfrm_idx);
                if (!lm) {
                    continue;
                }
                if (lm->will_be_erased()) {
                    continue;
                }

                const auto& keyfrm_desc = keyfrm->frm_obs_.descriptors_.row(keyfrm_idx);

                unsigned int best_hamm_dist = MAX_HAMMING_DIST;
                int best_frm_idx = -1;
                unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;

                for (const auto frm_idx : frm_indices) {
                    if (matched_lms_in_frm.at(frm_idx)) {
                        continue;
                    }

                    if (check_orientation_ && std::abs(util::angle::diff(keyfrm->frm_obs_.undist_keypts_.at(keyfrm_idx).angle, frm.frm_obs_.undist_keypts_.at(frm_idx).angle)) > 30.0) {
                        continue;
                    }

                    const auto& frm_desc = frm.frm_obs_.descriptors_.row(frm_idx);

                    const auto hamm_dist = compute_descriptor_distance_32(keyfrm_desc, frm_desc);

                    if (hamm_dist < best_hamm_dist) {
                        second_best_hamm_dist = best_hamm_dist;
                        best_hamm_dist = hamm_dist;
                        best_frm_idx = frm_idx;
                    }
                    else if (hamm_dist < second_best_hamm_dist) {
                        second_best_hamm_dist = hamm_dist;
                    }
                }

                if (HAMMING_DIST_THR_LOW < best_hamm_dist) {
                    continue;
                }

                // Ratio test
                if (lowe_ratio_ * second_best_hamm_dist < static_cast<float>(best_hamm_dist)) {
                    continue;
                }

                matched_lms_in_frm.at(best_frm_idx) = lm;

                ++num_matches;
            }

            ++keyfrm_itr;
            ++frm_itr;
        }
        else if (keyfrm_itr->first < frm_itr->first) {
            // Since the node number of the keyframe is smaller, increment the iterator until the node numbers match
            keyfrm_itr = keyfrm->bow_feat_vec_.lower_bound(frm_itr->first);
        }
        else {
            // Since the node number of the frame is smaller, increment the iterator until the node numbers match
            frm_itr = frm.bow_feat_vec_.lower_bound(keyfrm_itr->first);
        }
    }

    return num_matches;
}

unsigned int bow_tree::match_keyframes(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2, std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_1) const {
    unsigned int num_matches = 0;

    const auto keyfrm_1_lms = keyfrm_1->get_landmarks();
    const auto keyfrm_2_lms = keyfrm_2->get_landmarks();

    matched_lms_in_keyfrm_1 = std::vector<std::shared_ptr<data::landmark>>(keyfrm_1_lms.size(), nullptr);

    // Set 'true' if a keypoint in keyframe 2 is associated to the keypoint in keyframe 1
    // NOTE: the size matches the number of the keypoints in keyframe 2
    std::vector<bool> is_already_matched_in_keyfrm_2(keyfrm_2_lms.size(), false);

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
                // Ignore if the keypoint is not associated any 3D points
                // (because this function is used for Sim3 estimation)
                auto& lm_1 = keyfrm_1_lms.at(idx_1);
                if (!lm_1) {
                    continue;
                }
                if (lm_1->will_be_erased()) {
                    continue;
                }

                const auto& desc_1 = keyfrm_1->frm_obs_.descriptors_.row(idx_1);

                unsigned int best_hamm_dist = MAX_HAMMING_DIST;
                int best_idx_2 = -1;
                unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;

                for (const auto idx_2 : keyfrm_2_indices) {
                    // Ignore if the keypoint is not associated any 3D points
                    // (because this function is used for Sim3 estimation)
                    auto& lm_2 = keyfrm_2_lms.at(idx_2);
                    if (!lm_2) {
                        continue;
                    }
                    if (lm_2->will_be_erased()) {
                        continue;
                    }

                    if (is_already_matched_in_keyfrm_2.at(idx_2)) {
                        continue;
                    }

                    if (check_orientation_ && std::abs(util::angle::diff(keyfrm_1->frm_obs_.undist_keypts_.at(idx_1).angle, keyfrm_2->frm_obs_.undist_keypts_.at(idx_2).angle)) > 30.0) {
                        continue;
                    }

                    const auto& desc_2 = keyfrm_2->frm_obs_.descriptors_.row(idx_2);

                    const auto hamm_dist = compute_descriptor_distance_32(desc_1, desc_2);

                    if (hamm_dist < best_hamm_dist) {
                        second_best_hamm_dist = best_hamm_dist;
                        best_hamm_dist = hamm_dist;
                        best_idx_2 = idx_2;
                    }
                    else if (hamm_dist < second_best_hamm_dist) {
                        second_best_hamm_dist = hamm_dist;
                    }
                }

                if (HAMMING_DIST_THR_LOW < best_hamm_dist) {
                    continue;
                }

                // Ratio test
                if (lowe_ratio_ * second_best_hamm_dist < static_cast<float>(best_hamm_dist)) {
                    continue;
                }

                // Record the matching information
                // The index of keyframe 1 matches the best index 2 of keyframe 2
                matched_lms_in_keyfrm_1.at(idx_1) = keyfrm_2_lms.at(best_idx_2);
                // The best index of keyframe 2 already matches the keypoint of keyframe 1
                is_already_matched_in_keyfrm_2.at(best_idx_2) = true;

                num_matches++;
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

    return num_matches;
}

} // namespace match
} // namespace stella_vslam
