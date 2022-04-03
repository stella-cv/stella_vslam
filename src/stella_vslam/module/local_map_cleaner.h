#ifndef STELLA_VSLAM_MODULE_LOCAL_MAP_CLEANER_H
#define STELLA_VSLAM_MODULE_LOCAL_MAP_CLEANER_H

#include <list>
#include <memory>

namespace stella_vslam {

namespace data {
class keyframe;
class landmark;
class bow_database;
class map_database;
} // namespace data

namespace module {

class local_map_cleaner {
public:
    /**
     * Constructor
     */
    explicit local_map_cleaner(data::map_database* map_db, data::bow_database* bow_db, double redundant_obs_ratio_thr = 0.9);

    /**
     * Destructor
     */
    ~local_map_cleaner() = default;

    /**
     * Set the origin keyframe ID
     */
    void set_origin_keyframe_id(const unsigned int id) {
        origin_keyfrm_id_ = id;
    }

    /**
     * Add fresh landmark to check their redundancy
     */
    void add_fresh_landmark(std::shared_ptr<data::landmark>& lm) {
        fresh_landmarks_.push_back(lm);
    }

    /**
     * Reset the buffer
     */
    void reset();

    /**
     * Remove redundant landmarks
     */
    unsigned int remove_redundant_landmarks(const unsigned int cur_keyfrm_id, const bool depth_is_available);

    /**
     * Remove redundant keyframes
     */
    unsigned int remove_redundant_keyframes(const std::shared_ptr<data::keyframe>& cur_keyfrm) const;

    /**
     * Count the valid and the redundant observations in the specified keyframe
     */
    void count_redundant_observations(const std::shared_ptr<data::keyframe>& keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const;

private:
    //! map database
    data::map_database* map_db_ = nullptr;
    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //!
    double redundant_obs_ratio_thr_;

    //! origin keyframe ID
    unsigned int origin_keyfrm_id_ = 0;

    //! fresh landmarks to check their redundancy
    std::list<std::shared_ptr<data::landmark>> fresh_landmarks_;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_LOCAL_MAP_CLEANER_H
