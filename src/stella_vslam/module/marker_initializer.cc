#include "stella_vslam/module/marker_initializer.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/marker.h"
#include "stella_vslam/marker_model/base.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace module {

void marker_initializer::check_marker_initialization(data::marker& mkr, size_t needed_observations_for_initialization) {
    if (mkr.initialized_before_) {
        return;
    }

    auto id = mkr.id_;
    if (mkr.observations_.size() < needed_observations_for_initialization) {
        spdlog::debug("Not using marker {} yet, not enough keyframes ({}, need {})", id, mkr.observations_.size(), needed_observations_for_initialization);
        return;
    }

    std::vector<std::shared_ptr<data::keyframe>> valid_keyframes;
    for (const auto& id_keyfrm : mkr.observations_) {
        const auto& keyfrm = id_keyfrm.second;
        if (!keyfrm)
            continue;
        if (keyfrm->markers_2d_.find(id) == keyfrm->markers_2d_.end()) {
            spdlog::warn("Couldn't find 2D marker {} in keyframe that should have it", id);
            continue;
        }

        valid_keyframes.push_back(keyfrm);
    }

    if (valid_keyframes.size() < needed_observations_for_initialization) {
        spdlog::debug("Not using marker {} yet, not enough valid keyframes ({}, need {})", id, valid_keyframes.size(), needed_observations_for_initialization);
        return;
    }

    // Ok, initialize the positions
    eigen_alloc_vector<Vec3_t> corners_sum(4);
    for (auto& v : corners_sum) {
        v = {0.0, 0.0, 0.0};
    }

    for (const auto& keyfrm : valid_keyframes) {
        auto kf_pose = keyfrm->get_pose_wc();
        auto m2d_it = keyfrm->markers_2d_.find(id);
        assert(m2d_it != keyfrm->markers_2d_.end()); // We've checked this before
        auto& m2d = m2d_it->second;

        if (!m2d.marker_model_) {
            spdlog::error("Need marker model to be set to initialize 3D marker pos from 2D corners");
            return;
        }

        auto corners = m2d.compute_corners_pos_w(kf_pose, m2d.marker_model_->corners_pos_);
        for (size_t i = 0; i < 4; i++) {
            corners_sum[i] += corners[i];
        }
    }

    for (size_t i = 0; i < 4; i++) {
        corners_sum[i] /= valid_keyframes.size();
    }

    mkr.set_corner_pos(corners_sum);
    mkr.initialized_before_ = true;

    spdlog::debug("Initialized corner positions for marker {} from {} keyframes", id, valid_keyframes.size());
}

} // namespace module
} // namespace stella_vslam
