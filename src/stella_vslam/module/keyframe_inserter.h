#ifndef STELLA_VSLAM_MODULE_KEYFRAME_INSERTER_H
#define STELLA_VSLAM_MODULE_KEYFRAME_INSERTER_H

#include "stella_vslam/camera/base.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"

#include <memory>

namespace stella_vslam {

class mapping_module;

namespace data {
class map_database;
} // namespace data

namespace module {

class keyframe_inserter {
public:
    explicit keyframe_inserter(const double max_interval = 1.0,
                               const double min_interval = 0.1,
                               const double max_distance = -1.0,
                               const double lms_ratio_thr_almost_all_lms_are_tracked = 0.9,
                               const double lms_ratio_thr_view_changed = 0.8,
                               const unsigned int enough_lms_thr = 100,
                               const bool wait_for_local_bundle_adjustment = false);

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
                                const unsigned int num_reliable_lms,
                                const data::keyframe& ref_keyfrm,
                                const unsigned int min_num_obs_thr) const;

    /**
     * Insert the new keyframe derived from the current frame
     */
    void insert_new_keyframe(data::map_database* map_db, data::frame& curr_frm);

private:
    std::shared_ptr<data::keyframe> create_new_keyframe(data::map_database* map_db, data::frame& curr_frm);

    //! mapping module
    mapping_module* mapper_ = nullptr;

    //! max interval to insert keyframe
    const double max_interval_ = 1.0;
    const double min_interval_ = 0.1;
    const double max_distance_ = -1.0;

    //! Ratio-threshold of "the number of 3D points observed in the current frame" / "that of 3D points observed in the last keyframe"
    const double lms_ratio_thr_almost_all_lms_are_tracked_ = 0.9;
    const double lms_ratio_thr_view_changed_ = 0.8;

    const unsigned int enough_lms_thr_ = 100;
    const bool wait_for_local_bundle_adjustment_ = false;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_KEYFRAME_INSERTER_H
