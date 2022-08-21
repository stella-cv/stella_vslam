#include "stella_vslam/mapping_module.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/marker.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/module/keyframe_inserter.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace module {

keyframe_inserter::keyframe_inserter(const double max_interval,
                                     const double min_interval,
                                     const double max_distance,
                                     const double lms_ratio_thr_almost_all_lms_are_tracked,
                                     const double lms_ratio_thr_view_changed,
                                     const unsigned int enough_lms_thr)
    : max_interval_(max_interval),
      min_interval_(min_interval),
      max_distance_(max_distance),
      lms_ratio_thr_almost_all_lms_are_tracked_(lms_ratio_thr_almost_all_lms_are_tracked),
      lms_ratio_thr_view_changed_(lms_ratio_thr_view_changed),
      enough_lms_thr_(enough_lms_thr) {}

keyframe_inserter::keyframe_inserter(const YAML::Node& yaml_node)
    : keyframe_inserter(yaml_node["max_interval"].as<double>(1.0),
                        yaml_node["min_interval"].as<double>(0.1),
                        yaml_node["max_distance"].as<double>(-1.0),
                        yaml_node["lms_ratio_thr_almost_all_lms_are_tracked"].as<double>(0.9),
                        yaml_node["lms_ratio_thr_view_changed"].as<double>(0.5),
                        yaml_node["enough_lms_thr"].as<unsigned int>(100)) {}

void keyframe_inserter::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void keyframe_inserter::reset() {
}

bool keyframe_inserter::new_keyframe_is_needed(data::map_database* map_db,
                                               const data::frame& curr_frm,
                                               const unsigned int num_tracked_lms,
                                               const unsigned int num_reliable_lms,
                                               const data::keyframe& ref_keyfrm,
                                               const unsigned int min_num_obs_thr) const {
    assert(mapper_);
    // Any keyframes are not able to be added when the mapping module stops
    if (mapper_->is_paused() || mapper_->pause_is_requested()) {
        return false;
    }

    auto last_inserted_keyfrm = map_db->get_last_inserted_keyframe();

    // Count the number of the 3D points that are observed from more than two keyframes
    const auto num_reliable_lms_ref = ref_keyfrm.get_num_tracked_landmarks(min_num_obs_thr);

    // When the mapping module skips localBA, it does not insert keyframes
    const auto mapper_is_skipping_localBA = mapper_->is_skipping_localBA();

    constexpr unsigned int num_enough_keyfrms_thr = 5;
    const bool enough_keyfrms = map_db->get_num_keyframes() > num_enough_keyfrms_thr;

    // New keyframe is needed if the time elapsed since the last keyframe insertion reaches the threshold
    bool max_interval_elapsed = false;
    if (max_interval_ > 0.0) {
        max_interval_elapsed = last_inserted_keyfrm && last_inserted_keyfrm->timestamp_ + max_interval_ <= curr_frm.timestamp_;
    }
    bool min_interval_elapsed = true;
    if (min_interval_ > 0.0) {
        min_interval_elapsed = !last_inserted_keyfrm || last_inserted_keyfrm->timestamp_ + min_interval_ <= curr_frm.timestamp_;
    }
    bool max_distance_traveled = false;
    if (max_distance_ > 0.0) {
        max_distance_traveled = last_inserted_keyfrm && (last_inserted_keyfrm->get_trans_wc() - curr_frm.get_trans_wc()).norm() > max_distance_;
    }
    // New keyframe is needed if the field-of-view of the current frame is changed a lot
    const bool view_changed = num_reliable_lms < num_reliable_lms_ref * lms_ratio_thr_view_changed_;
    // const bool view_changed = num_tracked_lms < num_tracked_lms_on_ref_keyfrm * lms_ratio_thr_view_changed_;
    const bool not_enough_lms = num_reliable_lms < enough_lms_thr_;

    // (Mandatory for keyframe insertion)
    // New keyframe is needed if the number of 3D points exceeds the threshold,
    // and concurrently the ratio of the reliable 3D points larger than the threshold ratio
    constexpr unsigned int num_tracked_lms_thr_unstable = 15;
    bool tracking_is_unstable = num_tracked_lms < num_tracked_lms_thr_unstable;
    bool almost_all_lms_are_tracked = num_reliable_lms > num_reliable_lms_ref * lms_ratio_thr_almost_all_lms_are_tracked_;
    SPDLOG_TRACE("keyframe_inserter: num_reliable_lms_ref={}", num_reliable_lms_ref);
    SPDLOG_TRACE("keyframe_inserter: num_reliable_lms={}", num_reliable_lms);
    SPDLOG_TRACE("keyframe_inserter: max_interval_elapsed={}", max_interval_elapsed);
    SPDLOG_TRACE("keyframe_inserter: max_distance_traveled={}", max_distance_traveled);
    SPDLOG_TRACE("keyframe_inserter: view_changed={}", view_changed);
    SPDLOG_TRACE("keyframe_inserter: not_enough_lms={}", not_enough_lms);
    SPDLOG_TRACE("keyframe_inserter: enough_keyfrms={}", enough_keyfrms);
    SPDLOG_TRACE("keyframe_inserter: min_interval_elapsed={}", min_interval_elapsed);
    SPDLOG_TRACE("keyframe_inserter: tracking_is_unstable={}", tracking_is_unstable);
    SPDLOG_TRACE("keyframe_inserter: almost_all_lms_are_tracked={}", almost_all_lms_are_tracked);
    SPDLOG_TRACE("keyframe_inserter: mapper_is_skipping_localBA={}", mapper_is_skipping_localBA);
    return (max_interval_elapsed || max_distance_traveled || view_changed || not_enough_lms)
           && (!enough_keyfrms || min_interval_elapsed)
           && !tracking_is_unstable
           && !almost_all_lms_are_tracked
           && !mapper_is_skipping_localBA;
}

std::shared_ptr<data::keyframe> keyframe_inserter::insert_new_keyframe(data::map_database* map_db,
                                                                       data::frame& curr_frm) {
    auto keyfrm = data::keyframe::make_keyframe(map_db->next_keyframe_id_++, curr_frm);
    keyfrm->update_landmarks();

    for (const auto& id_mkr2d : keyfrm->markers_2d_) {
        auto marker = map_db->get_marker(id_mkr2d.first);
        if (!marker) {
            // Create new marker
            auto mkr2d = id_mkr2d.second;
            eigen_alloc_vector<Vec3_t> corners_pos_w = mkr2d.compute_corners_pos_w(keyfrm->get_pose_wc(), mkr2d.marker_model_->corners_pos_);
            marker = std::make_shared<data::marker>(corners_pos_w, id_mkr2d.first, mkr2d.marker_model_);
            // add the marker to the map DB
            map_db->add_marker(marker);
        }
        // Set the association to the new marker
        keyfrm->add_marker(marker);
        marker->observations_.push_back(keyfrm);
    }

    // Queue up the keyframe to the mapping module
    if (!keyfrm->depth_is_available()) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // Save the valid depth and index pairs
    std::vector<std::pair<float, unsigned int>> depth_idx_pairs;
    depth_idx_pairs.reserve(curr_frm.frm_obs_.num_keypts_);
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.num_keypts_; ++idx) {
        assert(!curr_frm.frm_obs_.depths_.empty());
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
            const auto& lm = curr_frm.get_landmark(idx);
            if (lm) {
                assert(lm->has_observation());
                continue;
            }
        }

        // Stereo-triangulation can be performed if the 3D point is not yet associated to the keypoint index
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = std::make_shared<data::landmark>(map_db->next_landmark_id_++, pos_w, keyfrm);

        lm->connect_to_keyframe(keyfrm, idx);
        curr_frm.add_landmark(lm, idx);

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
}

} // namespace module
} // namespace stella_vslam
