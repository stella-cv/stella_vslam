#ifndef OPENVSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
#define OPENVSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H

namespace openvslam {

namespace data {
class map_database;
} // namespace data

namespace optimize {

class global_bundle_adjuster {
public:
    /**
     * Constructor
     * @param map_db
     * @param num_iter
     * @param use_huber_kernel
     */
    explicit global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter = 10, const bool use_huber_kernel = true);

    /**
     * Destructor
     */
    virtual ~global_bundle_adjuster() = default;

    void optimize_for_initialization(bool* const force_stop_flag = nullptr) const;

    /**
     * Perform optimization
     * @param optimized_keyfrm_ids
     * @param optimized_landmark_ids
     * @param lm_to_pos_w_after_global_BA
     * @param keyfrm_to_pose_cw_after_global_BA
     * @param force_stop_flag
     */
    void optimize(std::unordered_set<unsigned int>& optimized_keyfrm_ids,
                  std::unordered_set<unsigned int>& optimized_landmark_ids,
                  eigen_alloc_unord_map<unsigned int, Vec3_t>& lm_to_pos_w_after_global_BA,
                  eigen_alloc_unord_map<unsigned int, Mat44_t>& keyfrm_to_pose_cw_after_global_BA,
                  bool* const force_stop_flag = nullptr) const;

private:
    //! map database
    const data::map_database* map_db_;

    //! number of iterations of optimization
    unsigned int num_iter_;

    //! use Huber loss or not
    const bool use_huber_kernel_;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_H
