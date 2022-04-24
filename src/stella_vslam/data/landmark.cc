#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/match/base.h"

#include <nlohmann/json.hpp>

namespace stella_vslam {
namespace data {

std::atomic<unsigned int> landmark::next_id_{0};

landmark::landmark(const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm)
    : id_(next_id_++), first_keyfrm_id_(ref_keyfrm->id_), pos_w_(pos_w),
      ref_keyfrm_(ref_keyfrm) {}

landmark::landmark(const unsigned int id, const unsigned int first_keyfrm_id,
                   const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm,
                   const unsigned int num_visible, const unsigned int num_found)
    : id_(id), first_keyfrm_id_(first_keyfrm_id), pos_w_(pos_w), ref_keyfrm_(ref_keyfrm),
      num_observable_(num_visible), num_observed_(num_found) {}

void landmark::set_pos_in_world(const Vec3_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    pos_w_ = pos_w;
}

Vec3_t landmark::get_pos_in_world() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return pos_w_;
}

Vec3_t landmark::get_obs_mean_normal() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return mean_normal_;
}

std::shared_ptr<keyframe> landmark::get_ref_keyframe() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return ref_keyfrm_.lock();
}

void landmark::add_observation(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    if (observations_.count(keyfrm)) {
        return;
    }
    observations_[keyfrm] = idx;

    if (!keyfrm->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm->frm_obs_.stereo_x_right_.at(idx)) {
        num_observations_ += 2;
    }
    else {
        num_observations_ += 1;
    }
}

void landmark::erase_observation(map_database* map_db, const std::shared_ptr<keyframe>& keyfrm) {
    bool discard = false;
    {
        std::lock_guard<std::mutex> lock(mtx_observations_);

        if (observations_.count(keyfrm)) {
            int idx = observations_.at(keyfrm);
            if (!keyfrm->frm_obs_.stereo_x_right_.empty() && 0 <= keyfrm->frm_obs_.stereo_x_right_.at(idx)) {
                num_observations_ -= 2;
            }
            else {
                num_observations_ -= 1;
            }

            observations_.erase(keyfrm);

            if (observations_.empty()) {
                discard = true;
            }
            else if (ref_keyfrm_.lock() == keyfrm) {
                ref_keyfrm_ = observations_.begin()->first.lock();
            }
        }
    }

    if (discard) {
        prepare_for_erasing(map_db);
    }
}

landmark::observations_t landmark::get_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return observations_;
}

unsigned int landmark::num_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observations_;
}

bool landmark::has_observation() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return 0 < num_observations_;
}

int landmark::get_index_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    if (observations_.count(keyfrm)) {
        return observations_.at(keyfrm);
    }
    else {
        return -1;
    }
}

bool landmark::is_observed_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<bool>(observations_.count(keyfrm));
}

cv::Mat landmark::get_descriptor() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return descriptor_.clone();
}

void landmark::compute_descriptor() {
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        if (will_be_erased_) {
            return;
        }
        observations = observations_;
    }

    if (observations.empty()) {
        return;
    }

    // Append features of corresponding points
    std::vector<cv::Mat> descriptors;
    descriptors.reserve(observations.size());
    for (const auto& observation : observations) {
        auto keyfrm = observation.first.lock();
        const auto idx = observation.second;

        if (!keyfrm->will_be_erased()) {
            descriptors.push_back(keyfrm->frm_obs_.descriptors_.row(idx));
        }
    }

    // Get median of Hamming distance
    // Calculate all the Hamming distances between every pair of the features
    const auto num_descs = descriptors.size();
    std::vector<std::vector<unsigned int>> hamm_dists(num_descs, std::vector<unsigned int>(num_descs));
    for (unsigned int i = 0; i < num_descs; ++i) {
        hamm_dists.at(i).at(i) = 0;
        for (unsigned int j = i + 1; j < num_descs; ++j) {
            const auto dist = match::compute_descriptor_distance_32(descriptors.at(i), descriptors.at(j));
            hamm_dists.at(i).at(j) = dist;
            hamm_dists.at(j).at(i) = dist;
        }
    }

    // Get the nearest value to median
    unsigned int best_median_dist = match::MAX_HAMMING_DIST;
    unsigned int best_idx = 0;
    for (unsigned idx = 0; idx < num_descs; ++idx) {
        std::vector<unsigned int> partial_hamm_dists(hamm_dists.at(idx).begin(), hamm_dists.at(idx).begin() + num_descs);
        std::sort(partial_hamm_dists.begin(), partial_hamm_dists.end());
        const auto median_dist = partial_hamm_dists.at(static_cast<unsigned int>(0.5 * (num_descs - 1)));

        if (median_dist < best_median_dist) {
            best_median_dist = median_dist;
            best_idx = idx;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        descriptor_ = descriptors.at(best_idx).clone();
    }
}

void landmark::update_mean_normal_and_obs_scale_variance() {
    observations_t observations;
    std::shared_ptr<keyframe> ref_keyfrm = nullptr;
    Vec3_t pos_w;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_position_);
        if (will_be_erased_) {
            return;
        }
        observations = observations_;
        ref_keyfrm = ref_keyfrm_.lock();
        pos_w = pos_w_;
    }

    if (observations.empty()) {
        return;
    }

    Vec3_t mean_normal = Vec3_t::Zero();
    for (const auto& observation : observations) {
        auto keyfrm = observation.first.lock();
        const Vec3_t normal = pos_w_ - keyfrm->get_trans_wc();
        mean_normal = mean_normal + normal.normalized();
    }

    const Vec3_t vec_ref_keyfrm_to_lm = pos_w - ref_keyfrm->get_trans_wc();
    const auto dist_ref_keyfrm_to_lm = vec_ref_keyfrm_to_lm.norm();
    const auto scale_level = ref_keyfrm->frm_obs_.undist_keypts_.at(observations.at(ref_keyfrm)).octave;
    const auto scale_factor = ref_keyfrm->orb_params_->scale_factors_.at(scale_level);
    const auto num_scale_levels = ref_keyfrm->orb_params_->num_levels_;

    {
        std::lock_guard<std::mutex> lock3(mtx_position_);
        max_valid_dist_ = dist_ref_keyfrm_to_lm * scale_factor;
        min_valid_dist_ = max_valid_dist_ * ref_keyfrm->orb_params_->inv_scale_factors_.at(num_scale_levels - 1);
        mean_normal_ = mean_normal.normalized();
    }
}

float landmark::get_min_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return min_valid_dist_;
}

float landmark::get_max_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return max_valid_dist_;
}

unsigned int landmark::predict_scale_level(const float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) const {
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mtx_position_);
        ratio = max_valid_dist_ / cam_to_lm_dist;
    }

    const auto pred_scale_level = static_cast<int>(std::ceil(std::log(ratio) / log_scale_factor));
    if (pred_scale_level < 0) {
        return 0;
    }
    else if (num_scale_levels <= static_cast<unsigned int>(pred_scale_level)) {
        return num_scale_levels - 1;
    }
    else {
        return static_cast<unsigned int>(pred_scale_level);
    }
}

void landmark::prepare_for_erasing(map_database* map_db) {
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        observations = observations_;
        observations_.clear();
        will_be_erased_ = true;
    }

    for (const auto& keyfrm_and_idx : observations) {
        keyfrm_and_idx.first.lock()->erase_landmark_with_index(keyfrm_and_idx.second);
    }

    map_db->erase_landmark(this->id_);
}

bool landmark::will_be_erased() {
    return will_be_erased_;
}

void landmark::replace(std::shared_ptr<landmark> lm, data::map_database* map_db) {
    if (lm->id_ == this->id_) {
        return;
    }

    // 1. Erase this
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        observations = observations_;
    }

    prepare_for_erasing(map_db);

    // 2. Connect lm
    unsigned int num_observable, num_observed;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        num_observable = num_observable_;
        num_observed = num_observed_;
        replaced_ = lm;
    }

    for (const auto& keyfrm_and_idx : observations) {
        const auto keyfrm = keyfrm_and_idx.first.lock();
        keyfrm->add_landmark(lm, keyfrm_and_idx.second);
        lm->add_observation(keyfrm, keyfrm_and_idx.second);
    }

    lm->increase_num_observed(num_observed);
    lm->increase_num_observable(num_observable);
    lm->compute_descriptor();
}

std::shared_ptr<landmark> landmark::get_replaced() const {
    std::lock_guard<std::mutex> lock1(mtx_observations_);
    return replaced_;
}

void landmark::increase_num_observable(unsigned int num_observable) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observable_ += num_observable;
}

void landmark::increase_num_observed(unsigned int num_observed) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observed_ += num_observed;
}

float landmark::get_observed_ratio() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<float>(num_observed_) / num_observable_;
}

unsigned int landmark::get_num_observed() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observed_;
}

unsigned int landmark::get_num_observable() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observable_;
}

nlohmann::json landmark::to_json() const {
    return {{"1st_keyfrm", first_keyfrm_id_},
            {"pos_w", {pos_w_(0), pos_w_(1), pos_w_(2)}},
            {"ref_keyfrm", ref_keyfrm_.lock()->id_},
            {"n_vis", num_observable_},
            {"n_fnd", num_observed_}};
}

} // namespace data
} // namespace stella_vslam
