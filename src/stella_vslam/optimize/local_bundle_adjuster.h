#ifndef STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H
#define STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H

#include <memory>

namespace stella_vslam {

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace optimize {

class local_bundle_adjuster {
public:
    /**
     * Perform optimization
     * @param map_db
     * @param curr_keyfrm
     * @param force_stop_flag
     */
    virtual void optimize(data::map_database* map_db, const std::shared_ptr<data::keyframe>& curr_keyfrm, bool* const force_stop_flag) const = 0;
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H
