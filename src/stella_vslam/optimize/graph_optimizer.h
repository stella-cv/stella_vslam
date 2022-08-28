#ifndef STELLA_VSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H
#define STELLA_VSLAM_OPTIMIZE_GRAPH_OPTIMIZER_H

#include "stella_vslam/module/type.h"

#include <map>
#include <set>
#include <memory>

namespace stella_vslam {

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace optimize {

class graph_optimizer {
public:
    /**
     * Constructor
     * @param fix_scale
     */
    explicit graph_optimizer(const bool fix_scale);

    /**
     * Destructor
     */
    virtual ~graph_optimizer() = default;

    /**
     * Perform pose graph optimization
     * @param loop_keyfrm
     * @param curr_keyfrm
     * @param non_corrected_Sim3s
     * @param pre_corrected_Sim3s
     * @param loop_connections
     */
    void optimize(const std::shared_ptr<data::keyframe>& loop_keyfrm, const std::shared_ptr<data::keyframe>& curr_keyfrm,
                  const module::keyframe_Sim3_pairs_t& non_corrected_Sim3s,
                  const module::keyframe_Sim3_pairs_t& pre_corrected_Sim3s,
                  const std::map<std::shared_ptr<data::keyframe>, std::set<std::shared_ptr<data::keyframe>>>& loop_connections,
                  std::unordered_map<unsigned int, unsigned int>& found_lm_to_ref_keyfrm_id) const;

private:
    //! SE3 optimization or Sim3 optimization
    const bool fix_scale_;
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_GRAPH_OPTIMIZER_H
