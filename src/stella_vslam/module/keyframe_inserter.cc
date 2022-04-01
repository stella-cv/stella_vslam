#include "stella_vslam/mapping_module.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/module/keyframe_inserter.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace module {

keyframe_inserter::keyframe_inserter(const double max_interval,
                                     const double lms_ratio_thr_almost_all_lms_are_tracked,
                                     const double lms_ratio_thr_view_changed)
    : max_interval_(max_interval),
      lms_ratio_thr_almost_all_lms_are_tracked_(lms_ratio_thr_almost_all_lms_are_tracked),
      lms_ratio_thr_view_changed_(lms_ratio_thr_view_changed) {}

keyframe_inserter::keyframe_inserter(const YAML::Node& yaml_node)
    : keyframe_inserter(yaml_node["max_interval"].as<double>(1.0),
                        yaml_node["lms_ratio_thr_almost_all_lms_are_tracked"].as<double>(0.95),
                        yaml_node["lms_ratio_thr_view_changed"].as<double>(0.9)) {}

void keyframe_inserter::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void keyframe_inserter::reset() {
}

bool keyframe_inserter::new_keyframe_is_needed(data::map_database* map_db, const data::frame& curr_frm,
                                               const unsigned int num_tracked_lms,
                                               const data::keyframe& ref_keyfrm) const {
    assert(mapper_);
    // Any keyframes are not able to be added when the mapping module stops
    if (mapper_->is_paused() || mapper_->pause_is_requested()) {
        return false;
    }

    const auto num_keyfrms = map_db->get_num_keyframes();
    auto last_inserted_keyfrm = map_db->get_last_inserted_keyframe();

    // Count the number of the 3D points that are observed from more than two keyframes
    const unsigned int min_obs_thr = (3 <= num_keyfrms) ? 3 : 2;
    const auto num_reliable_lms = ref_keyfrm.get_num_tracked_landmarks(min_obs_thr);

    // When the mapping module skips localBA, it does not insert keyframes
    const auto mapper_is_skipping_localBA = mapper_->is_skipping_localBA();

    constexpr unsigned int num_tracked_lms_thr_unstable = 15;

    // New keyframe is needed if the time elapsed since the last keyframe insertion reaches the threshold
    const bool max_interval_elapsed = last_inserted_keyfrm && last_inserted_keyfrm->timestamp_ + max_interval_ <= curr_frm.timestamp_;
    // New keyframe is needed if the field-of-view of the current frame is changed a lot
    const bool view_changed = num_tracked_lms < num_reliable_lms * lms_ratio_thr_view_changed_;

    // (Mandatory for keyframe insertion)
    // New keyframe is needed if the number of 3D points exceeds the threshold,
    // and concurrently the ratio of the reliable 3D points larger than the threshold ratio
    bool tracking_is_unstable = num_tracked_lms < num_tracked_lms_thr_unstable;
    bool almost_all_lms_are_tracked = num_tracked_lms > num_reliable_lms * lms_ratio_thr_almost_all_lms_are_tracked_;

    return (max_interval_elapsed || view_changed) && !tracking_is_unstable && !almost_all_lms_are_tracked && !mapper_is_skipping_localBA;
}

std::shared_ptr<data::keyframe> keyframe_inserter::insert_new_keyframe(data::map_database* map_db,
                                                                       data::frame& curr_frm) {
    // Do not pause mapping_module to let this keyframe process
    if (!mapper_->prevent_pause_if_not_paused()) {
        // If it is already paused, exit
        return nullptr;
    }

    curr_frm.update_pose_params();
    auto keyfrm = data::keyframe::make_keyframe(curr_frm);

    // Queue up the keyframe to the mapping module
    if (!keyfrm->depth_is_avaliable()) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // Save the valid depth and index pairs
    std::vector<std::pair<float, unsigned int>> depth_idx_pairs;
    depth_idx_pairs.reserve(curr_frm.frm_obs_.num_keypts_);
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.num_keypts_; ++idx) {
        const auto depth = curr_frm.frm_obs_.depths_.at(idx);
        // Add if the depth is valid
        if (0 < depth) {
            depth_idx_pairs.emplace_back(std::make_pair(depth, idx));
        }
    }

    // Queue up the keyframe to the mapping module if any valid depth values don't exist
    if (depth_idx_pairs.empty()) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // Sort in order of distance to the camera
    std::sort(depth_idx_pairs.begin(), depth_idx_pairs.end());

    // Create 3D points by using a depth parameter
    constexpr unsigned int min_num_to_create = 100;
    for (unsigned int count = 0; count < depth_idx_pairs.size(); ++count) {
        const auto depth = depth_idx_pairs.at(count).first;
        const auto idx = depth_idx_pairs.at(count).second;

        // Stop adding a keyframe if the number of 3D points exceeds the minimal threshold,
        // and concurrently the depth value exceeds the threshold
        if (min_num_to_create < count && keyfrm->camera_->depth_thr_ < depth) {
            break;
        }

        // Stereo-triangulation cannot be performed if the 3D point has been already associated to the keypoint index
        {
            const auto& lm = curr_frm.landmarks_.at(idx);
            if (lm) {
                assert(lm->has_observation());
                continue;
            }
        }

        // Stereo-triangulation can be performed if the 3D point is not yet associated to the keypoint index
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = std::make_shared<data::landmark>(pos_w, keyfrm, map_db);

        lm->add_observation(keyfrm, idx);
        keyfrm->add_landmark(lm, idx);
        curr_frm.landmarks_.at(idx) = lm;

        lm->compute_descriptor();
        lm->update_mean_normal_and_obs_scale_variance();

        map_db->add_landmark(lm);
    }

    // Queue up the keyframe to the mapping module
    queue_keyframe(keyfrm);
    return keyfrm;
}

void keyframe_inserter::queue_keyframe(const std::shared_ptr<data::keyframe>& keyfrm) {
    mapper_->queue_keyframe(keyfrm);
    mapper_->stop_prevent_pause();
}

} // namespace module
} // namespace stella_vslam
