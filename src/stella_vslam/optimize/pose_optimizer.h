#ifndef STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_H
#define STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_H

#include "stella_vslam/type.h"

namespace stella_vslam {

namespace data {
class frame;
struct frame_observation;
class keyframe;
} // namespace data

namespace camera {
class base;
} // namespace camera

namespace feature {
struct orb_params;
} // namespace feature

namespace optimize {

class pose_optimizer {
public:
    /**
     * Constructor
     * @param num_trials
     * @param num_each_iter
     */
    explicit pose_optimizer(const unsigned int num_trials = 4, const unsigned int num_each_iter = 10);

    /**
     * Destructor
     */
    virtual ~pose_optimizer() = default;

    /**
     * Perform pose optimization
     * @param frm
     * @return
     */
    unsigned int optimize(const data::frame& frm, g2o::SE3Quat& optimized_pose, std::vector<bool>& outlier_flags) const;
    unsigned int optimize(const data::keyframe* keyfrm, g2o::SE3Quat& optimized_pose, std::vector<bool>& outlier_flags) const;

    unsigned int optimize(const Mat44_t& cam_pose_cw, const data::frame_observation& frm_obs,
                          const feature::orb_params* orb_params,
                          const camera::base* camera,
                          const std::vector<std::shared_ptr<data::landmark>>& landmarks,
                          g2o::SE3Quat& optimized_pose,
                          std::vector<bool>& outlier_flags) const;

private:
    //! robust optimizationの試行回数
    const unsigned int num_trials_ = 4;

    //! 毎回のoptimizationのiteration回数
    const unsigned int num_each_iter_ = 10;
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_H
