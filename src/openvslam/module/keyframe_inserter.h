#ifndef OPENVSLAM_MODULE_KEYFRAME_INSERTER_H
#define OPENVSLAM_MODULE_KEYFRAME_INSERTER_H

#include "openvslam/camera/base.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"

#include <memory>

namespace openvslam {

class mapping_module;

namespace data {
class map_database;
} // namespace data

namespace module {

class keyframe_inserter {
public:
    explicit keyframe_inserter(const double max_interval = 1.0,
                               const double lms_ratio_thr_almost_all_lms_are_tracked = 0.95,
                               const double lms_ratio_thr_view_changed = 0.9);

    explicit keyframe_inserter(const YAML::Node& yaml_node);

    virtual ~keyframe_inserter() = default;

    void set_mapping_module(mapping_module* mapper);

    void reset();

    /**
     * Check the new keyframe is needed or not
     */
    bool new_keyframe_is_needed(data::map_database* map_db,
                                const data::frame& curr_frm,
                                const unsigned int num_tracked_lms,
                                const data::keyframe& ref_keyfrm) const;

    /**
     * Insert the new keyframe derived from the current frame
     */
    std::shared_ptr<data::keyframe> insert_new_keyframe(data::map_database* map_db, data::frame& curr_frm);

private:
    /**
     * Queue the new keyframe to the mapping module
     */
    void queue_keyframe(const std::shared_ptr<data::keyframe>& keyfrm);

    //! mapping module
    mapping_module* mapper_ = nullptr;

    //! max interval to insert keyframe
    const double max_interval_;

    //! Ratio-threshold of "the number of 3D points observed in the current frame" / "that of 3D points observed in the last keyframe"
    const double lms_ratio_thr_almost_all_lms_are_tracked_ = 0.95;
    const double lms_ratio_thr_view_changed_ = 0.9;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_KEYFRAME_INSERTER_H
