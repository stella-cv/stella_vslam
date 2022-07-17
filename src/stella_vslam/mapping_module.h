#ifndef STELLA_VSLAM_MAPPING_MODULE_H
#define STELLA_VSLAM_MAPPING_MODULE_H

#include "stella_vslam/config.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/module/local_map_cleaner.h"
#include "stella_vslam/optimize/local_bundle_adjuster.h"
#include "stella_vslam/data/bow_vocabulary_fwd.h"

#include <mutex>
#include <atomic>
#include <memory>
#include <future>

#include <yaml-cpp/yaml.h>

namespace stella_vslam {

class config;
class tracking_module;
class global_optimization_module;

namespace camera {
class base;
} // namespace camera

namespace data {
class keyframe;
class bow_database;
class map_database;
} // namespace data

class mapping_module {
public:
    //! Constructor
    mapping_module(const YAML::Node& yaml_node, data::map_database* map_db, data::bow_database* bow_db, data::bow_vocabulary* bow_vocab);

    //! Destructor
    ~mapping_module();

    //! Set the tracking module
    void set_tracking_module(tracking_module* tracker);

    //! Set the global optimization module
    void set_global_optimization_module(global_optimization_module* global_optimizer);

    //-----------------------------------------
    // main process

    //! Run main loop of the mapping module
    void run();

    //! Queue a keyframe to process the mapping
    void queue_keyframe(const std::shared_ptr<data::keyframe>& keyfrm);

    //! Check if keyframe is queued
    bool keyframe_is_queued() const;

    //! Get the number of queued keyframes
    unsigned int get_num_queued_keyframes() const;

    //! True when no keyframes are being processed
    bool is_idle() const;

    //! If the size of the queue exceeds this threshold, skip the localBA
    bool is_skipping_localBA() const;

    //-----------------------------------------
    // management for reset process

    //! Request to reset the mapping module
    std::shared_future<void> async_reset();

    //-----------------------------------------
    // management for pause process

    //! Request to pause the mapping module
    std::shared_future<void> async_pause();

    //! Check if the mapping module is requested to be paused or not
    bool pause_is_requested() const;

    //! Check if the mapping module is paused or not
    bool is_paused() const;

    //! Resume the mapping module
    void resume();

    //-----------------------------------------
    // management for terminate process

    //! Request to terminate the mapping module
    std::shared_future<void> async_terminate();

    //! Check if the mapping module is terminated or not
    bool is_terminated() const;

    //-----------------------------------------
    // management for local BA

    //! Abort the local BA externally
    //! (NOTE: this function does not wait for abort)
    void abort_local_BA();

#ifdef DETERMINISTIC
    //-----------------------------------------
    // management for synchronization with tracking

    //! Signal that processing is done
    std::condition_variable processing_cv_;

    //! Mutex for blocking tracking until we're done
    std::mutex mtx_processing_;
#endif

private:
    //-----------------------------------------
    // main process

    //! Create and extend the map with the new keyframe
    void mapping_with_new_keyframe();

    //! Store the new keyframe to the map database
    void store_new_keyframe();

    //! Create new landmarks using neighbor keyframes
    void create_new_landmarks(std::atomic<bool>& abort_create_new_landmarks);

    //! Triangulate landmarks between the keyframes 1 and 2
    void triangulate_with_two_keyframes(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2,
                                        const std::vector<std::pair<unsigned int, unsigned int>>& matches);

    //! Update the new keyframe
    void update_new_keyframe();

    //! Fuse duplicated landmarks between current keyframe and covisibility keyframes
    void fuse_landmark_duplication(const std::vector<std::shared_ptr<data::keyframe>>& fuse_tgt_keyfrms,
                                   nondeterministic::unordered_map<std::shared_ptr<data::landmark>, std::shared_ptr<data::landmark>>& replaced_lms);

    //! Set is_idle (True when no keyframes are being processed.)
    void set_is_idle(const bool is_idle);

    //-----------------------------------------
    // management for reset process

    //! mutex for access to reset procedure
    mutable std::mutex mtx_reset_;

    //! promise for reset
    std::promise<void> promise_reset_;

    //! future for reset
    std::shared_future<void> future_reset_;

    //! Check and execute reset
    bool reset_is_requested() const;

    //! Reset the variables
    void reset();

    //! flag which indicates whether reset is requested or not
    bool reset_is_requested_ = false;

    //-----------------------------------------
    // management for pause process

    //! mutex for access to pause procedure
    mutable std::mutex mtx_pause_;

    //! promise for pause
    std::promise<void> promise_pause_;

    //! future for pause
    std::shared_future<void> future_pause_;

    //! Pause the mapping module
    void pause();

    //! flag which indicates termination is requested or not
    bool pause_is_requested_ = false;
    //! flag which indicates whether the main loop is paused or not
    bool is_paused_ = false;

    //-----------------------------------------
    // management for terminate process

    //! mutex for access to terminate procedure
    mutable std::mutex mtx_terminate_;

    //! promise for terminate
    std::promise<void> promise_terminate_;

    //! future for terminate
    std::shared_future<void> future_terminate_;

    //! Check if termination is requested or not
    bool terminate_is_requested() const;

    //! Raise the flag which indicates the main loop has been already terminated
    void terminate();

    //! flag which indicates termination is requested or not
    bool terminate_is_requested_ = false;
    //! flag which indicates whether the main loop is terminated or not
    bool is_terminated_ = true;

    //-----------------------------------------
    // modules

    //! tracking module
    tracking_module* tracker_ = nullptr;
    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;

    //! local map cleaner
    std::unique_ptr<module::local_map_cleaner> local_map_cleaner_ = nullptr;

    //-----------------------------------------
    // database

    //! map database
    data::map_database* map_db_ = nullptr;

    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //! BoW vocabulary
    data::bow_vocabulary* bow_vocab_ = nullptr;

    //-----------------------------------------
    // keyframe queue

    //! mutex for access to keyframe queue
    mutable std::mutex mtx_keyfrm_queue_;

    //! queue for keyframes
    std::list<std::shared_ptr<data::keyframe>> keyfrms_queue_;

    //-----------------------------------------
    // optimizer

    //! local bundle adjuster
    std::unique_ptr<optimize::local_bundle_adjuster> local_bundle_adjuster_ = nullptr;

    //! bridge flag to abort local BA
    bool abort_local_BA_ = false;

    //-----------------------------------------
    // others

    //! True when no keyframes are being processed
    std::atomic<bool> is_idle_{true};

    //! current keyframe which is used in the current mapping
    std::shared_ptr<data::keyframe> cur_keyfrm_ = nullptr;

    //-----------------------------------------
    // configurations

    //! If true, use baseline_dist_thr_ratio_ in mapping_module::create_new_landmarks. Otherwise use baseline_dist_thr_.
    bool use_baseline_dist_thr_ratio_ = true;

    //! Create new landmarks if the baseline distance is greater than the median depth times baseline_dist_thr_ratio_ of the reference keyframe.
    double baseline_dist_thr_ratio_ = 0.02;

    //! Create new landmarks if the baseline distance is greater than baseline_dist_thr_ of the reference keyframe.
    double baseline_dist_thr_ = 1.0;

    //! If the size of the queue exceeds this threshold, skip the localBA
    const unsigned int queue_threshold_ = 2;

    //! if true, enable interruption of landmark generation
    const bool enable_interruption_of_landmark_generation_ = true;

    //! if true, enable interruption before local BA
    const bool enable_interruption_before_local_BA_ = true;

    //! Number of keyframes used for landmark generation
    const unsigned int num_covisibilities_for_landmark_generation_ = 10;

    //! Number of keyframes used for landmark fusion
    const unsigned int num_covisibilities_for_landmark_fusion_ = 10;
};

} // namespace stella_vslam

#endif // STELLA_VSLAM_MAPPING_MODULE_H
