#ifndef STELLA_VSLAM_DATA_LANDMARK_H
#define STELLA_VSLAM_DATA_LANDMARK_H

#include "stella_vslam/type.h"

#include <map>
#include <mutex>
#include <atomic>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <nlohmann/json_fwd.hpp>

namespace stella_vslam {
namespace data {

class frame;

class keyframe;

class map_database;

class landmark {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using observations_t = std::map<std::weak_ptr<keyframe>, unsigned int, std::owner_less<std::weak_ptr<keyframe>>>;

    //! constructor
    landmark(const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm);

    //! constructor for map loading with computing parameters which can be recomputed
    landmark(const unsigned int id, const unsigned int first_keyfrm_id,
             const Vec3_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm,
             const unsigned int num_visible, const unsigned int num_found);

    //! set world coordinates of this landmark
    void set_pos_in_world(const Vec3_t& pos_w);
    //! get world coordinates of this landmark
    Vec3_t get_pos_in_world() const;

    //! get mean normalized vector of keyframe->lm vectors, for keyframes such that observe the 3D point.
    Vec3_t get_obs_mean_normal() const;
    //! get reference keyframe, a keyframe at the creation of a given 3D point
    std::shared_ptr<keyframe> get_ref_keyframe() const;

    //! add observation
    void add_observation(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx);
    //! erase observation
    void erase_observation(map_database* map_db, const std::shared_ptr<keyframe>& keyfrm);

    //! get observations (keyframe and keypoint idx)
    observations_t get_observations() const;
    //! get number of observations
    unsigned int num_observations() const;
    //! whether this landmark is observed from more than zero keyframes
    bool has_observation() const;

    //! get index of associated keypoint in the specified keyframe
    int get_index_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const;
    //! whether this landmark is observed in the specified keyframe
    bool is_observed_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const;

    //! check the distance between landmark and camera is in ORB scale variance
    inline bool is_inside_in_orb_scale(const float cam_to_lm_dist) const {
        const float max_dist = this->get_max_valid_distance();
        const float min_dist = this->get_min_valid_distance();
        return (min_dist <= cam_to_lm_dist && cam_to_lm_dist <= max_dist);
    }

    //! get representative descriptor
    cv::Mat get_descriptor() const;

    //! compute representative descriptor
    void compute_descriptor();

    //! update observation mean normal and ORB scale variance
    void update_mean_normal_and_obs_scale_variance();

    //! get max valid distance between landmark and camera
    float get_min_valid_distance() const;
    //! get min valid distance between landmark and camera
    float get_max_valid_distance() const;

    //! predict scale level assuming this landmark is observed in the specified frame/keyframe
    unsigned int predict_scale_level(const float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) const;

    //! erase this landmark from database
    void prepare_for_erasing(map_database* map_db);
    //! whether this landmark will be erased shortly or not
    bool will_be_erased();

    //! replace this with specified landmark
    void replace(std::shared_ptr<landmark> lm, data::map_database* map_db);
    //! get replace landmark
    std::shared_ptr<landmark> get_replaced() const;

    void increase_num_observable(unsigned int num_observable = 1);
    void increase_num_observed(unsigned int num_observed = 1);
    unsigned int get_num_observed() const;
    unsigned int get_num_observable() const;
    float get_observed_ratio() const;

    //! encode landmark information as JSON
    nlohmann::json to_json() const;

public:
    unsigned int id_;
    static std::atomic<unsigned int> next_id_;
    unsigned int first_keyfrm_id_ = 0;
    unsigned int num_observations_ = 0;

private:
    //! world coordinates of this landmark
    Vec3_t pos_w_;

    //! observations (keyframe and keypoint index)
    observations_t observations_;

    //! Normalized average vector (unit vector) of keyframe->lm, for keyframes such that observe the 3D point.
    Vec3_t mean_normal_ = Vec3_t::Zero();

    //! representative descriptor
    cv::Mat descriptor_;

    //! reference keyframe
    std::weak_ptr<keyframe> ref_keyfrm_;

    // track counter
    unsigned int num_observable_ = 1;
    unsigned int num_observed_ = 1;

    //! this landmark will be erased shortly or not
    std::atomic<bool> will_be_erased_{false};

    //! replace this landmark with below
    std::shared_ptr<landmark> replaced_ = nullptr;

    // ORB scale variances
    //! max valid distance between landmark and camera
    float min_valid_dist_ = 0;
    //! min valid distance between landmark and camera
    float max_valid_dist_ = 0;

    mutable std::mutex mtx_position_;
    mutable std::mutex mtx_observations_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_LANDMARK_H
