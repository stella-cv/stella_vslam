#ifndef STELLA_VSLAM_DATA_FRAME_H
#define STELLA_VSLAM_DATA_FRAME_H

#include "stella_vslam/type.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/feature/orb_params.h"
#include "stella_vslam/util/converter.h"
#include "stella_vslam/data/frame_observation.h"
#include "stella_vslam/data/bow_vocabulary.h"
#include "stella_vslam/data/marker2d.h"

#include <vector>
#include <atomic>
#include <memory>

#include <opencv2/core.hpp>
#include <Eigen/Core>

#ifdef USE_DBOW2
#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#else
#include <fbow/bow_vector.h>
#include <fbow/bow_feat_vector.h>
#endif

namespace stella_vslam {

namespace camera {
class base;
} // namespace camera

namespace feature {
class orb_extractor;
struct orb_params;
} // namespace feature

namespace data {

class keyframe;
class landmark;

class frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame() = default;

    bool operator==(const frame& frm) { return this->id_ == frm.id_; }
    bool operator!=(const frame& frm) { return !(*this == frm); }

    /**
     * Constructor for monocular frame
     * @param timestamp
     * @param camera
     * @param orb_params
     * @param frm_obs
     * @param markers_2d
     */
    frame(const double timestamp, camera::base* camera, feature::orb_params* orb_params,
          const frame_observation frm_obs, const std::unordered_map<unsigned int, marker2d>& markers_2d);

    /**
     * Set camera pose and refresh rotation and translation
     * @param cam_pose_cw
     */
    void set_cam_pose(const Mat44_t& cam_pose_cw);

    /**
     * Set camera pose and refresh rotation and translation
     * @param cam_pose_cw
     */
    void set_cam_pose(const g2o::SE3Quat& cam_pose_cw);

    /**
     * Get camera pose
     */
    Mat44_t get_cam_pose() const;

    /**
     * Get the inverse of the camera pose
     */
    Mat44_t get_cam_pose_inv() const;

    /**
     * Update rotation and translation using cam_pose_cw_
     */
    void update_pose_params();

    /**
     * Get camera center
     * @return
     */
    Vec3_t get_cam_center() const;

    /**
     * Get inverse of rotation
     * @return
     */
    Mat33_t get_rotation_inv() const;

    /**
     * Returns true if BoW has been computed.
     */
    bool bow_is_available() const;

    /**
     * Compute BoW representation
     */
    void compute_bow(bow_vocabulary* bow_vocab);

    /**
     * Check observability of the landmark
     */
    bool can_observe(const std::shared_ptr<landmark>& lm, const float ray_cos_thr,
                     Vec2_t& reproj, float& x_right, unsigned int& pred_scale_level) const;

    /**
     * Get keypoint indices in the cell which reference point is located
     * @param ref_x
     * @param ref_y
     * @param margin
     * @param min_level
     * @param max_level
     * @return
     */
    std::vector<unsigned int> get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level = -1, const int max_level = -1) const;

    /**
     * Perform stereo triangulation of the keypoint
     * @param idx
     * @return
     */
    Vec3_t triangulate_stereo(const unsigned int idx) const;

    //! current frame ID
    unsigned int id_;

    //! next frame ID
    static std::atomic<unsigned int> next_id_;

    //! timestamp
    double timestamp_;

    //! camera model
    camera::base* camera_ = nullptr;

    //! ORB scale pyramid information
    const feature::orb_params* orb_params_ = nullptr;

    //! constant observations
    frame_observation frm_obs_;

    //! markers 2D (ID to marker2d map)
    std::unordered_map<unsigned int, marker2d> markers_2d_;

    //! BoW features (DBoW2 or FBoW)
#ifdef USE_DBOW2
    DBoW2::BowVector bow_vec_;
    DBoW2::FeatureVector bow_feat_vec_;
#else
    fbow::BoWVector bow_vec_;
    fbow::BoWFeatVector bow_feat_vec_;
#endif

    //! landmarks, whose nullptr indicates no-association
    std::vector<std::shared_ptr<landmark>> landmarks_;

    //! camera pose: world -> camera
    bool cam_pose_cw_is_valid_ = false;
    Mat44_t cam_pose_cw_;

    //! reference keyframe for tracking
    std::shared_ptr<keyframe> ref_keyfrm_ = nullptr;

private:
    //! Camera pose
    //! rotation: world -> camera
    Mat33_t rot_cw_;
    //! translation: world -> camera
    Vec3_t trans_cw_;
    //! rotation: camera -> world
    Mat33_t rot_wc_;
    //! translation: camera -> world
    Vec3_t cam_center_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_FRAME_H
