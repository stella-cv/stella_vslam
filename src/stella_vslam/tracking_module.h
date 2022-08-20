#ifndef STELLA_VSLAM_TRACKING_MODULE_H
#define STELLA_VSLAM_TRACKING_MODULE_H

#include "stella_vslam/type.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/module/initializer.h"
#include "stella_vslam/module/relocalizer.h"
#include "stella_vslam/module/keyframe_inserter.h"
#include "stella_vslam/module/frame_tracker.h"

#include <mutex>
#include <memory>
#include <future>

#include <opencv2/core/types.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace stella_vslam {

class system;
class mapping_module;
class global_optimization_module;

namespace data {
class map_database;
class bow_database;
} // namespace data

// tracker state
enum class tracker_state_t {
    Initializing,
    Tracking,
    Lost
};

struct pose_request {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool mode_2d_;
    Mat44_t pose_cw_;
    Vec3_t normal_vector_;
};

class tracking_module {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    tracking_module(const std::shared_ptr<config>& cfg, camera::base* camera, data::map_database* map_db,
                    data::bow_vocabulary* bow_vocab, data::bow_database* bow_db);

    //! Destructor
    ~tracking_module();

    //! Set the mapping module
    void set_mapping_module(mapping_module* mapper);

    //! Set the global optimization module
    void set_global_optimization_module(global_optimization_module* global_optimizer);

    //-----------------------------------------
    // interfaces for mapping module and global optimization module

    //! Replace the landmarks in last frame
    void replace_landmarks_in_last_frm(nondeterministic::unordered_map<std::shared_ptr<data::landmark>, std::shared_ptr<data::landmark>>& replaced_lms);

    //-----------------------------------------
    // interfaces

    //! Main stream of the tracking module
    std::shared_ptr<Mat44_t> feed_frame(data::frame frame);

    //! Request to update the pose to a given one.
    //! Return failure in case if previous request was not finished yet.
    bool request_relocalize_by_pose(const Mat44_t& pose_cw);
    bool request_relocalize_by_pose_2d(const Mat44_t& pose_cw, const Vec3_t& normal_vector);

    //-----------------------------------------
    // management for reset process

    //! Reset the databases
    void reset();

    //-----------------------------------------
    // management for stop keyframe insertion process

    //! Request to stop keyframe insertion in tracking module
    std::future<void> async_stop_keyframe_insertion();

    //! Request to start keyframe insertion in tracking module
    std::future<void> async_start_keyframe_insertion();

    //-----------------------------------------
    // management for pause process

    //! Request to pause the tracking module
    std::shared_future<void> async_pause();

    //! Check if the pause of the tracking module is requested or not
    bool pause_is_requested() const;

    //! Check if the tracking module is paused or not
    bool is_paused() const;

    //! Resume the tracking module
    void resume();

    //-----------------------------------------
    // configurations

    //! camera model
    camera::base* camera_;

    //! closest keyframes thresholds (by distance and angle) to relocalize with when updating by pose
    double reloc_distance_threshold_ = 0.2;
    double reloc_angle_threshold_ = 0.45;

    //! If true, automatically try to relocalize when lost
    bool enable_auto_relocalization_ = true;

    //! If true, use robust_matcher for relocalization request
    bool use_robust_matcher_for_relocalization_request_ = false;

    //-----------------------------------------
    // variables

    //! latest tracking state
    tracker_state_t tracking_state_ = tracker_state_t::Initializing;

    //! current frame and its image
    data::frame curr_frm_;

protected:
    //-----------------------------------------
    // tracking processes

    //! Try to initialize with the current frame
    bool initialize();

    //! Main stream of the tracking module
    bool track(bool relocalization_is_needed);

    //! Track the current frame
    bool track_current_frame();

    //! Relocalization by pose
    bool relocalize_by_pose(const pose_request& request);

    //! Get close keyframes
    std::vector<std::shared_ptr<data::keyframe>> get_close_keyframes(const pose_request& request);

    //! Update the motion model using the current and last frames
    void update_motion_model();

    //! Update the camera pose of the last frame
    void update_last_frame();

    //! Optimize the camera pose of the current frame
    bool optimize_current_frame_with_local_map(unsigned int& num_tracked_lms,
                                               unsigned int& num_reliable_lms,
                                               const unsigned int min_num_obs_thr);

    //! Update the local map
    void update_local_map();

    //! Acquire more 2D-3D matches using initial camera pose estimation
    void search_local_landmarks();

    //! Check the new keyframe is needed or not
    bool new_keyframe_is_needed(unsigned int num_tracked_lms,
                                unsigned int num_reliable_lms,
                                const unsigned int min_num_obs_thr) const;

    //! Insert the new keyframe derived from the current frame
    void insert_new_keyframe();

    //! mapping module
    mapping_module* mapper_ = nullptr;
    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;

    //! map_database
    data::map_database* map_db_ = nullptr;

    // Bag of Words
    //! BoW vocabulary
    data::bow_vocabulary* bow_vocab_ = nullptr;
    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //! initializer
    module::initializer initializer_;

    //! frame tracker for current frame
    const module::frame_tracker frame_tracker_;

    //! relocalizer
    module::relocalizer relocalizer_;

    //! pose optimizer
    const optimize::pose_optimizer pose_optimizer_;

    //! keyframe inserter
    module::keyframe_inserter keyfrm_inserter_;

    //! local keyframes
    std::vector<std::shared_ptr<data::keyframe>> local_keyfrms_;
    //! local landmarks
    std::vector<std::shared_ptr<data::landmark>> local_landmarks_;

    //! last frame
    data::frame last_frm_;

    //! mutex for pause process
    mutable std::mutex mtx_last_frm_;

    //! ID of latest frame which succeeded in relocalization
    unsigned int last_reloc_frm_id_ = 0;
    //! timestamp of latest frame which succeeded in relocalization
    double last_reloc_frm_timestamp_ = 0.0;

    //! motion model
    Mat44_t twist_;
    //! motion model is valid or not
    bool twist_is_valid_ = false;

    //! current camera pose from reference keyframe
    //! (to update last camera pose at the beginning of each tracking)
    Mat44_t last_cam_pose_from_ref_keyfrm_;

    //-----------------------------------------
    // management for stop_keyframe_insertion process

    //! mutex for stop_keyframe_insertion process
    mutable std::mutex mtx_stop_keyframe_insertion_;

    bool is_stopped_keyframe_insertion_ = false;

    //-----------------------------------------
    // management for pause process

    //! mutex for pause process
    mutable std::mutex mtx_pause_;

    //! promise for pause
    std::promise<void> promise_pause_;

    //! future for pause
    std::shared_future<void> future_pause_;

    //! Check the request frame and pause the tracking module
    bool pause_if_requested();

    //! the tracking module is paused or not
    bool is_paused_ = false;

    //! Pause of the tracking module is requested or not
    bool pause_is_requested_ = false;

    //-----------------------------------------
    // force relocalization

    //! Mutex for update pose request into given position
    mutable std::mutex mtx_relocalize_by_pose_request_;
    //! Update into a given position is requested or not
    bool relocalize_by_pose_is_requested();
    //! Get requested for relocalization pose
    pose_request& get_relocalize_by_pose_request();
    //! Finish update request. Returns true in case of request was made.
    void finish_relocalize_by_pose_request();
    //! Indicator of update pose request
    bool relocalize_by_pose_is_requested_ = false;
    //! Requested pose to update
    pose_request relocalize_by_pose_request_;
};

} // namespace stella_vslam

#endif // STELLA_VSLAM_TRACKING_MODULE_H
