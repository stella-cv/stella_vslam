#ifndef STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
#define STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H

namespace stella_vslam {

namespace data {
class map_database;
} // namespace data

namespace optimize {

class global_bundle_adjuster {
public:
    /**
     * Constructor
     * @param num_iter
     * @param use_huber_kernel
     */
    explicit global_bundle_adjuster(const unsigned int num_iter = 10, const bool use_huber_kernel = true);

    /**
     * Destructor
     */
    virtual ~global_bundle_adjuster() = default;

    void optimize_for_initialization(const std::vector<std::shared_ptr<data::keyframe>>& keyfrms,
                                     const std::vector<std::shared_ptr<data::landmark>>& lms,
                                     const std::vector<std::shared_ptr<data::marker>>& markers,
                                     bool* const force_stop_flag = nullptr) const;

    /**
     * Perform optimization
     * @param keyfrms
     * @param optimized_keyfrm_ids
     * @param optimized_landmark_ids
     * @param lm_to_pos_w_after_global_BA
     * @param keyfrm_to_pose_cw_after_global_BA
     * @param force_stop_flag
     * @return false if aborted
     */
    bool optimize(const std::vector<std::shared_ptr<data::keyframe>>& keyfrms,
                  std::unordered_set<unsigned int>& optimized_keyfrm_ids,
                  std::unordered_set<unsigned int>& optimized_landmark_ids,
                  eigen_alloc_unord_map<unsigned int, Vec3_t>& lm_to_pos_w_after_global_BA,
                  eigen_alloc_unord_map<unsigned int, Mat44_t>& keyfrm_to_pose_cw_after_global_BA,
                  bool* const force_stop_flag = nullptr) const;

private:
    //! number of iterations of optimization
    unsigned int num_iter_;

    //! use Huber loss or not
    const bool use_huber_kernel_;
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
