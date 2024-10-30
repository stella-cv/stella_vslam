#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/data/marker.h"
#include "stella_vslam/publish/map_publisher.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace publish {

map_publisher::map_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db)
    : cfg_(cfg), map_db_(map_db) {
    spdlog::debug("CONSTRUCT: publish::map_publisher");
}

map_publisher::~map_publisher() {
    spdlog::debug("DESTRUCT: publish::map_publisher");
}

void map_publisher::set_current_cam_pose(const Mat44_t& cam_pose_cw) {
    std::lock_guard<std::mutex> lock(mtx_cam_pose_);
    cam_pose_cw_ = cam_pose_cw;
}

Mat44_t map_publisher::get_current_cam_pose() {
    std::lock_guard<std::mutex> lock(mtx_cam_pose_);
    return cam_pose_cw_;
}

unsigned int map_publisher::get_keyframes(std::vector<std::shared_ptr<data::keyframe>>& all_keyfrms) {
    auto roots = map_db_->get_spanning_roots();
    if (roots.empty()) {
        return 0;
    }
    all_keyfrms = roots.back()->graph_node_->get_keyframes_from_root();
    return map_db_->get_num_keyframes();
}

unsigned int map_publisher::get_landmarks(std::vector<std::shared_ptr<data::landmark>>& all_landmarks,
                                          std::set<std::shared_ptr<data::landmark>>& local_landmarks) {
    auto roots = map_db_->get_spanning_roots();
    if (roots.empty()) {
        return 0;
    }
    auto keyfrms = roots.back()->graph_node_->get_keyframes_from_root();
    std::unordered_set<unsigned int> already_found_landmark_ids;
    all_landmarks.clear();
    for (const auto& keyfrm : keyfrms) {
        for (const auto& lm : keyfrm->get_landmarks()) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }
            if (already_found_landmark_ids.count(lm->id_)) {
                continue;
            }

            already_found_landmark_ids.insert(lm->id_);
            all_landmarks.push_back(lm);
        }
    }

    const auto _local_landmarks = map_db_->get_local_landmarks();
    local_landmarks = std::set<std::shared_ptr<data::landmark>>(_local_landmarks.begin(), _local_landmarks.end());
    return map_db_->get_num_landmarks();
}

unsigned int map_publisher::get_markers(std::vector<std::shared_ptr<data::marker>>& all_markers) {
    all_markers = map_db_->get_all_markers();
    return all_markers.size();
}

} // namespace publish
} // namespace stella_vslam
