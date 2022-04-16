#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/module/local_map_cleaner.h"

namespace stella_vslam {
namespace module {

local_map_cleaner::local_map_cleaner(const YAML::Node& yaml_node, data::map_database* map_db, data::bow_database* bow_db)
    : map_db_(map_db), bow_db_(bow_db),
      redundant_obs_ratio_thr_(yaml_node["redundant_obs_ratio_thr"].as<double>(0.9)),
      observed_ratio_thr_(yaml_node["observed_ratio_thr"].as<double>(0.3)),
      num_obs_thr_(yaml_node["num_obs_thr"].as<unsigned int>(2)),
      num_reliable_keyfrms_(yaml_node["num_reliable_keyfrms"].as<unsigned int>(2)),
      desired_valid_obs_(yaml_node["desired_valid_obs"].as<unsigned int>(0)),
      num_obs_keyfrms_thr_(yaml_node["num_obs_keyfrms_thr"].as<unsigned int>(10)) {}

void local_map_cleaner::reset() {
    fresh_landmarks_.clear();
}

unsigned int local_map_cleaner::remove_redundant_landmarks(const unsigned int cur_keyfrm_id) {
    // states of observed landmarks
    enum class lm_state_t { Valid,
                            Invalid,
                            NotClear };

    unsigned int num_removed = 0;
    auto iter = fresh_landmarks_.begin();
    while (iter != fresh_landmarks_.end()) {
        const auto& lm = *iter;

        // decide the state of lms the buffer
        auto lm_state = lm_state_t::NotClear;
        if (lm->will_be_erased()) {
            // in case `lm` will be erased
            // remove `lm` from the buffer
            lm_state = lm_state_t::Valid;
        }
        else if (lm->get_observed_ratio() < observed_ratio_thr_) {
            // if `lm` is not reliable
            // remove `lm` from the buffer and the database
            lm_state = lm_state_t::Invalid;
        }
        else if (num_reliable_keyfrms_ + lm->first_keyfrm_id_ <= cur_keyfrm_id
                 && lm->num_observations() <= num_obs_thr_) {
            // if the number of the observers of `lm` is small after some keyframes were inserted
            // remove `lm` from the buffer and the database
            lm_state = lm_state_t::Invalid;
        }
        else if (num_reliable_keyfrms_ + lm->first_keyfrm_id_ < cur_keyfrm_id) {
            // if the number of the observers of `lm` is sufficient after some keyframes were inserted
            // remove `lm` from the buffer
            lm_state = lm_state_t::Valid;
        }

        // select to remove `lm` according to the state
        if (lm_state == lm_state_t::Valid) {
            iter = fresh_landmarks_.erase(iter);
        }
        else if (lm_state == lm_state_t::Invalid) {
            ++num_removed;
            lm->prepare_for_erasing(map_db_);
            iter = fresh_landmarks_.erase(iter);
        }
        else {
            // hold decision because the state is NotClear
            iter++;
        }
    }

    return num_removed;
}

unsigned int local_map_cleaner::remove_redundant_keyframes(const std::shared_ptr<data::keyframe>& cur_keyfrm) const {
    // window size not to remove
    constexpr unsigned int window_size_not_to_remove = 2;
    // if the redundancy ratio of observations is larger than this threshold,
    // the corresponding keyframe will be erased
    unsigned int num_removed = 0;
    // check redundancy for each of the covisibilities
    const auto cur_covisibilities = cur_keyfrm->graph_node_->get_covisibilities();
    for (const auto& covisibility : cur_covisibilities) {
        // cannot remove the origin
        if (covisibility->id_ == origin_keyfrm_id_) {
            continue;
        }
        // cannot remove the recent keyframe(s)
        if (covisibility->id_ <= cur_keyfrm->id_
            && cur_keyfrm->id_ <= covisibility->id_ + window_size_not_to_remove) {
            continue;
        }

        // count the number of redundant observations (num_redundant_obs) and valid observations (num_valid_obs)
        // for the covisibility
        unsigned int num_redundant_obs = 0;
        unsigned int num_valid_obs = 0;
        count_redundant_observations(covisibility, num_valid_obs, num_redundant_obs);

        // Remove redundant connections
        unsigned int num_removed_connection = 0;
        if (desired_valid_obs_ > 0 && num_valid_obs > desired_valid_obs_) {
            auto lms = covisibility->get_landmarks();
            for (unsigned int idx = 0; idx < lms.size(); ++idx) {
                const auto& lm = lms.at(idx);
                if (!lm) {
                    continue;
                }
                if (lm->will_be_erased()) {
                    continue;
                }

                const auto num_obs_keyfrms = lm->num_observations();
                if (num_obs_keyfrms > num_obs_keyfrms_thr_) {
                    covisibility->erase_landmark_with_index(idx);
                    lm->erase_observation(map_db_, covisibility);
                    num_removed_connection++;
                    if (num_removed_connection >= num_valid_obs - desired_valid_obs_) {
                        break;
                    }
                }
            }
            covisibility->graph_node_->update_connections();
        }

        // if the redundant observation ratio of `covisibility` is larger than the threshold, it will be removed
        if (redundant_obs_ratio_thr_ <= static_cast<float>(num_redundant_obs) / num_valid_obs) {
            ++num_removed;
            covisibility->prepare_for_erasing(map_db_, bow_db_);
        }
    }

    return num_removed;
}

void local_map_cleaner::count_redundant_observations(const std::shared_ptr<data::keyframe>& keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const {
    // if the number of keyframes that observes the landmark with more reliable scale than the specified keyframe does,
    // it is considered as redundant
    constexpr unsigned int num_better_obs_thr = 3;

    num_valid_obs = 0;
    num_redundant_obs = 0;

    const auto landmarks = keyfrm->get_landmarks();
    for (unsigned int idx = 0; idx < landmarks.size(); ++idx) {
        const auto& lm = landmarks.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // if depth is within the valid range, it won't be considered
        if (keyfrm->depth_is_available()) {
            assert(!keyfrm->frm_obs_.depths_.empty());
            const auto depth = keyfrm->frm_obs_.depths_.at(idx);
            if (depth < 0.0 || keyfrm->camera_->depth_thr_ < depth) {
                continue;
            }
        }

        ++num_valid_obs;

        // if the number of the obs is smaller than the threshold, cannot remote the observers
        if (lm->num_observations() <= num_better_obs_thr) {
            continue;
        }

        // `keyfrm` observes `lm` with the scale level `scale_level`
        const auto scale_level = keyfrm->frm_obs_.undist_keypts_.at(idx).octave;
        // get observers of `lm`
        const auto observations = lm->get_observations();

        bool obs_by_keyfrm_is_redundant = false;

        // the number of the keyframes that observe `lm` with the more reliable (closer) scale
        unsigned int num_better_obs = 0;

        for (const auto& obs : observations) {
            const auto ngh_keyfrm = obs.first.lock();
            if (*ngh_keyfrm == *keyfrm) {
                continue;
            }

            // `ngh_keyfrm` observes `lm` with the scale level `ngh_scale_level`
            const auto ngh_scale_level = ngh_keyfrm->frm_obs_.undist_keypts_.at(obs.second).octave;

            // compare the scale levels
            if (ngh_scale_level <= scale_level + 1) {
                // the observation by `ngh_keyfrm` is more reliable than `keyfrm`
                ++num_better_obs;
                if (num_better_obs_thr <= num_better_obs) {
                    // if the number of the better observations is greater than the threshold,
                    // consider the observation of `lm` by `keyfrm` is redundant
                    obs_by_keyfrm_is_redundant = true;
                    break;
                }
            }
        }

        if (obs_by_keyfrm_is_redundant) {
            ++num_redundant_obs;
        }
    }
}

} // namespace module
} // namespace stella_vslam
