#ifndef STELLA_VSLAM_MODULE_FRAME_TRACKER_H
#define STELLA_VSLAM_MODULE_FRAME_TRACKER_H

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/pose_optimizer.h"

#include <memory>

namespace stella_vslam {

namespace camera {
class base;
} // namespace camera

namespace data {
class frame;
class keyframe;
class bow_database;
} // namespace data

namespace module {

class frame_tracker {
public:
    explicit frame_tracker(camera::base* camera, const unsigned int num_matches_thr = 20);

    bool motion_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity,
                            std::unordered_set<unsigned int>& outlier_ids) const;

    bool bow_match_based_track(data::frame& curr_frm, const data::frame& last_frm, const std::shared_ptr<data::keyframe>& ref_keyfrm,
                               std::unordered_set<unsigned int>& outlier_ids) const;

    bool robust_match_based_track(data::frame& curr_frm, const data::frame& last_frm, const std::shared_ptr<data::keyframe>& ref_keyfrm,
                                  std::unordered_set<unsigned int>& outlier_ids) const;

private:
    unsigned int discard_outliers(data::frame& curr_frm, std::unordered_set<unsigned int>& outlier_ids) const;

    const camera::base* camera_;
    const unsigned int num_matches_thr_;

    const optimize::pose_optimizer pose_optimizer_;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_FRAME_TRACKER_H
