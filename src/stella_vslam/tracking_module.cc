#include "stella_vslam/config.h"
#include "stella_vslam/system.h"
#include "stella_vslam/tracking_module.h"
#include "stella_vslam/mapping_module.h"
#include "stella_vslam/global_optimization_module.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/data/bow_database.h"
#include "stella_vslam/match/projection.h"
#include "stella_vslam/module/local_map_updater.h"
#include "stella_vslam/optimize/pose_optimizer_factory.h"
#include "stella_vslam/util/yaml.h"

#include <chrono>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace stella_vslam {

tracking_module::tracking_module(const std::shared_ptr<config>& cfg, camera::base* camera, data::map_database* map_db,
                                 data::bow_vocabulary* bow_vocab, data::bow_database* bow_db)
    : camera_(camera),
      reloc_distance_threshold_(util::yaml_optional_ref(cfg->yaml_node_, "Tracking")["reloc_distance_threshold"].as<double>(0.2)),
      reloc_angle_threshold_(util::yaml_optional_ref(cfg->yaml_node_, "Tracking")["reloc_angle_threshold"].as<double>(0.45)),
      init_retry_threshold_time_(util::yaml_optional_ref(cfg->yaml_node_, "Tracking")["init_retry_threshold_time"].as<double>(5.0)),
      enable_auto_relocalization_(util::yaml_optional_ref(cfg->yaml_node_, "Tracking")["enable_auto_relocalization"].as<bool>(true)),
      enable_temporal_keyframe_only_tracking_(util::yaml_optional_ref(cfg->yaml_node_, "Tracking")["enable_temporal_keyframe_only_tracking"].as<bool>(false)),
      use_robust_matcher_for_relocalization_request_(util::yaml_optional_ref(cfg->yaml_node_, "Tracking")["use_robust_matcher_for_relocalization_request"].as<bool>(false)),
      max_num_local_keyfrms_(util::yaml_optional_ref(cfg->yaml_node_, "Tracking")["max_num_local_keyfrms"].as<unsigned int>(60)),
      map_db_(map_db), bow_vocab_(bow_vocab), bow_db_(bow_db),
      initializer_(map_db, bow_db, util::yaml_optional_ref(cfg->yaml_node_, "Initializer")),
      pose_optimizer_(optimize::pose_optimizer_factory::create(util::yaml_optional_ref(cfg->yaml_node_, "Tracking"))),
      frame_tracker_(camera_, pose_optimizer_, 10, initializer_.get_use_fixed_seed()),
      relocalizer_(pose_optimizer_, util::yaml_optional_ref(cfg->yaml_node_, "Relocalizer")),
      keyfrm_inserter_(util::yaml_optional_ref(cfg->yaml_node_, "KeyframeInserter")) {
    spdlog::debug("CONSTRUCT: tracking_module");
}

tracking_module::~tracking_module() {
    spdlog::debug("DESTRUCT: tracking_module");
}

void tracking_module::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
    keyfrm_inserter_.set_mapping_module(mapper);
}

void tracking_module::set_global_optimization_module(global_optimization_module* global_optimizer) {
    global_optimizer_ = global_optimizer;
}

bool tracking_module::request_relocalize_by_pose(const Mat44_t& pose_cw) {
    std::lock_guard<std::mutex> lock(mtx_relocalize_by_pose_request_);
    if (relocalize_by_pose_is_requested_) {
        spdlog::warn("Can not process new pose update request while previous was not finished");
        return false;
    }
    relocalize_by_pose_is_requested_ = true;
    relocalize_by_pose_request_.mode_2d_ = false;
    relocalize_by_pose_request_.pose_cw_ = pose_cw;
    return true;
}

bool tracking_module::request_relocalize_by_pose_2d(const Mat44_t& pose_cw, const Vec3_t& normal_vector) {
    std::lock_guard<std::mutex> lock(mtx_relocalize_by_pose_request_);
    if (relocalize_by_pose_is_requested_) {
        spdlog::warn("Can not process new pose update request while previous was not finished");
        return false;
    }
    relocalize_by_pose_is_requested_ = true;
    relocalize_by_pose_request_.mode_2d_ = true;
    relocalize_by_pose_request_.pose_cw_ = pose_cw;
    relocalize_by_pose_request_.normal_vector_ = normal_vector;
    return true;
}

bool tracking_module::relocalize_by_pose_is_requested() {
    std::lock_guard<std::mutex> lock(mtx_relocalize_by_pose_request_);
    return relocalize_by_pose_is_requested_;
}

pose_request& tracking_module::get_relocalize_by_pose_request() {
    std::lock_guard<std::mutex> lock(mtx_relocalize_by_pose_request_);
    return relocalize_by_pose_request_;
}

void tracking_module::finish_relocalize_by_pose_request() {
    std::lock_guard<std::mutex> lock(mtx_relocalize_by_pose_request_);
    relocalize_by_pose_is_requested_ = false;
}

void tracking_module::reset() {
    spdlog::info("resetting system");

    initializer_.reset();
    keyfrm_inserter_.reset();

    auto future_mapper_reset = mapper_->async_reset();
    auto future_global_optimizer_reset = global_optimizer_->async_reset();
    future_mapper_reset.get();
    future_global_optimizer_reset.get();

    bow_db_->clear();
    map_db_->clear();

    data::frame::next_id_ = 0;

    last_reloc_frm_id_ = 0;
    last_reloc_frm_timestamp_ = 0.0;

    tracking_state_ = tracker_state_t::Initializing;
}

std::shared_ptr<Mat44_t> tracking_module::feed_frame(data::frame curr_frm) {
    // check if pause is requested
    pause_if_requested();
    while (is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    curr_frm_ = curr_frm;

    bool succeeded = false;
    if (tracking_state_ == tracker_state_t::Initializing) {
        succeeded = initialize();
    }
    else {
        std::lock_guard<std::mutex> lock(mtx_stop_keyframe_insertion_);
        bool relocalization_is_needed = tracking_state_ == tracker_state_t::Lost;
        SPDLOG_TRACE("tracking_module: start tracking");
        unsigned int num_tracked_lms = 0;
        unsigned int num_reliable_lms = 0;
        const unsigned int min_num_obs_thr = (3 <= map_db_->get_num_keyframes()) ? 3 : 2;
        succeeded = track(relocalization_is_needed, num_tracked_lms, num_reliable_lms, min_num_obs_thr);

        // check to insert the new keyframe derived from the current frame
        if (succeeded && !is_stopped_keyframe_insertion_ && new_keyframe_is_needed(num_tracked_lms, num_reliable_lms, min_num_obs_thr)) {
            keyfrm_inserter_.insert_new_keyframe(map_db_, curr_frm_);
        }
    }

    // state transition
    if (succeeded) {
        tracking_state_ = tracker_state_t::Tracking;
    }
    else if (tracking_state_ == tracker_state_t::Tracking) {
        tracking_state_ = tracker_state_t::Lost;

        spdlog::info("tracking lost: frame {}", curr_frm_.id_);
        // if tracking is failed within init_retry_threshold_time_ sec after initialization, reset the system
        if (!mapper_->is_paused() && curr_frm_.timestamp_ - initializer_.get_initial_frame_timestamp() < init_retry_threshold_time_) {
            spdlog::info("tracking lost within {} sec after initialization", init_retry_threshold_time_);
            reset();
            return nullptr;
        }
    }

    std::shared_ptr<Mat44_t> cam_pose_wc = nullptr;
    // store the relative pose from the reference keyframe to the current frame
    // to update the camera pose at the beginning of the next tracking process
    if (curr_frm_.pose_is_valid()) {
        last_cam_pose_from_ref_keyfrm_ = curr_frm_.get_pose_cw() * curr_frm_.ref_keyfrm_->get_pose_wc();
        cam_pose_wc = std::allocate_shared<Mat44_t>(Eigen::aligned_allocator<Mat44_t>(), curr_frm_.get_pose_wc());
    }

    // update last frame
    SPDLOG_TRACE("tracking_module: update last frame (curr_frm_={})", curr_frm_.id_);
    {
        std::lock_guard<std::mutex> lock(mtx_last_frm_);
        last_frm_ = curr_frm_;
    }
    SPDLOG_TRACE("tracking_module: finish tracking");

    return cam_pose_wc;
}

bool tracking_module::track(bool relocalization_is_needed,
                            unsigned int& num_tracked_lms,
                            unsigned int& num_reliable_lms,
                            const unsigned int min_num_obs_thr) {
    // LOCK the map database
    std::lock_guard<std::mutex> lock1(data::map_database::mtx_database_);
    std::lock_guard<std::mutex> lock2(mtx_last_frm_);

    // update the camera pose of the last frame
    // because the mapping module might optimize the camera pose of the last frame's reference keyframe
    SPDLOG_TRACE("tracking_module: update the camera pose of the last frame (curr_frm_={})", curr_frm_.id_);
    update_last_frame();

    // set the reference keyframe of the current frame
    curr_frm_.ref_keyfrm_ = last_frm_.ref_keyfrm_;

    bool succeeded = false;
    if (relocalize_by_pose_is_requested()) {
        // Force relocalization by pose
        succeeded = relocalize_by_pose(get_relocalize_by_pose_request());
    }
    else if (!relocalization_is_needed) {
        SPDLOG_TRACE("tracking_module: track_current_frame (curr_frm_={})", curr_frm_.id_);
        succeeded = track_current_frame();
    }
    else if (enable_auto_relocalization_) {
        // Compute the BoW representations to perform relocalization
        SPDLOG_TRACE("tracking_module: Compute the BoW representations to perform relocalization (curr_frm_={})", curr_frm_.id_);
        if (!curr_frm_.bow_is_available()) {
            curr_frm_.compute_bow(bow_vocab_);
        }
        // try to relocalize
        SPDLOG_TRACE("tracking_module: try to relocalize (curr_frm_={})", curr_frm_.id_);
        succeeded = relocalizer_.relocalize(bow_db_, curr_frm_);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
            last_reloc_frm_timestamp_ = curr_frm_.timestamp_;
        }
    }

    // update the local map and optimize current camera pose
    if (succeeded) {
        succeeded = track_local_map(num_tracked_lms, num_reliable_lms, min_num_obs_thr);
    }

    // update the local map and optimize current camera pose without temporal keyframes
    unsigned int fixed_keyframe_id_threshold = map_db_->get_fixed_keyframe_id_threshold();
    if (fixed_keyframe_id_threshold > 0 && succeeded) {
        succeeded = track_local_map_without_temporal_keyframes(num_tracked_lms, num_reliable_lms, min_num_obs_thr, fixed_keyframe_id_threshold);
    }

    // update the motion model
    if (succeeded) {
        SPDLOG_TRACE("tracking_module: update_motion_model (curr_frm_={})", curr_frm_.id_);
        update_motion_model();
    }

    // update the frame statistics
    SPDLOG_TRACE("tracking_module: update_frame_statistics (curr_frm_={})", curr_frm_.id_);
    map_db_->update_frame_statistics(curr_frm_, !succeeded);

    return succeeded;
}

bool tracking_module::track_local_map(unsigned int& num_tracked_lms,
                                      unsigned int& num_reliable_lms,
                                      const unsigned int min_num_obs_thr) {
    bool succeeded = false;
    SPDLOG_TRACE("tracking_module: update_local_map (curr_frm_={})", curr_frm_.id_);
    succeeded = update_local_map();

    if (succeeded) {
        succeeded = search_local_landmarks();
    }

    if (succeeded) {
        SPDLOG_TRACE("tracking_module: optimize_current_frame_with_local_map (curr_frm_={})", curr_frm_.id_);
        succeeded = optimize_current_frame_with_local_map(num_tracked_lms, num_reliable_lms, min_num_obs_thr);
    }

    if (!succeeded) {
        spdlog::info("local map tracking failed (curr_frm_={})", curr_frm_.id_);
    }
    return succeeded;
}

bool tracking_module::track_local_map_without_temporal_keyframes(unsigned int& num_tracked_lms,
                                                                 unsigned int& num_reliable_lms,
                                                                 const unsigned int min_num_obs_thr,
                                                                 const unsigned int fixed_keyframe_id_threshold) {
    bool succeeded = false;
    SPDLOG_TRACE("tracking_module: update_local_map without temporal keyframes (curr_frm_={})", curr_frm_.id_);
    succeeded = update_local_map(fixed_keyframe_id_threshold);

    if (succeeded) {
        succeeded = search_local_landmarks();
    }

    if (enable_temporal_keyframe_only_tracking_ && !succeeded) {
        SPDLOG_TRACE("temporal keyframe only tracking (curr_frm_={})", curr_frm_.id_);
        return true;
    }

    if (succeeded) {
        SPDLOG_TRACE("tracking_module: optimize_current_frame_with_local_map without temporal keyframes (curr_frm_={})", curr_frm_.id_);
        succeeded = optimize_current_frame_with_local_map(num_tracked_lms, num_reliable_lms, min_num_obs_thr);
    }

    if (!succeeded) {
        spdlog::info("local map tracking failed (curr_frm_={})", curr_frm_.id_);
    }
    return succeeded;
}

bool tracking_module::initialize() {
    {
        // LOCK the map database
        std::lock_guard<std::mutex> lock1(data::map_database::mtx_database_);
        std::lock_guard<std::mutex> lock2(mtx_stop_keyframe_insertion_);

        // try to initialize with the current frame
        initializer_.initialize(camera_->setup_type_, bow_vocab_, curr_frm_);
    }

    // if map building was failed -> reset the map database
    if (initializer_.get_state() == module::initializer_state_t::Wrong) {
        reset();
        return false;
    }

    // if initializing was failed -> try to initialize with the next frame
    if (initializer_.get_state() != module::initializer_state_t::Succeeded) {
        return false;
    }

    // pass all of the keyframes to the mapping module
    assert(!is_stopped_keyframe_insertion_);
    for (const auto& keyfrm : curr_frm_.ref_keyfrm_->graph_node_->get_keyframes_from_root()) {
        auto future = mapper_->async_add_keyframe(keyfrm);
        future.get();
    }

    // succeeded
    return true;
}

bool tracking_module::track_current_frame() {
    bool succeeded = false;

    // Tracking mode
    if (twist_is_valid_ && last_reloc_frm_id_ + 2 < curr_frm_.id_) {
        // if the motion model is valid
        succeeded = frame_tracker_.motion_based_track(curr_frm_, last_frm_, twist_);
    }
    if (!succeeded) {
        // Compute the BoW representations to perform the BoW match
        if (!curr_frm_.bow_is_available()) {
            curr_frm_.compute_bow(bow_vocab_);
        }
        succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, curr_frm_.ref_keyfrm_);
    }
    if (!succeeded) {
        succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, curr_frm_.ref_keyfrm_);
    }

    return succeeded;
}

bool tracking_module::relocalize_by_pose(const pose_request& request) {
    bool succeeded = false;
    curr_frm_.set_pose_cw(request.pose_cw_);

    if (!curr_frm_.bow_is_available()) {
        curr_frm_.compute_bow(bow_vocab_);
    }
    const auto candidates = get_close_keyframes(request);
    for (const auto& candidate : candidates) {
        spdlog::debug("relocalize_by_pose: candidate = {}", candidate->id_);
    }

    if (!candidates.empty()) {
        succeeded = relocalizer_.reloc_by_candidates(curr_frm_, candidates, use_robust_matcher_for_relocalization_request_);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
            last_reloc_frm_timestamp_ = curr_frm_.timestamp_;
        }
    }
    else {
        curr_frm_.invalidate_pose();
    }
    finish_relocalize_by_pose_request();
    return succeeded;
}

std::vector<std::shared_ptr<data::keyframe>> tracking_module::get_close_keyframes(const pose_request& request) {
    if (request.mode_2d_) {
        return map_db_->get_close_keyframes_2d(
            request.pose_cw_,
            request.normal_vector_,
            reloc_distance_threshold_,
            reloc_angle_threshold_);
    }
    else {
        return map_db_->get_close_keyframes(
            request.pose_cw_,
            reloc_distance_threshold_,
            reloc_angle_threshold_);
    }
}

void tracking_module::update_motion_model() {
    if (last_frm_.pose_is_valid()) {
        Mat44_t last_frm_cam_pose_wc = Mat44_t::Identity();
        last_frm_cam_pose_wc.block<3, 3>(0, 0) = last_frm_.get_rot_wc();
        last_frm_cam_pose_wc.block<3, 1>(0, 3) = last_frm_.get_trans_wc();
        twist_is_valid_ = true;
        twist_ = curr_frm_.get_pose_cw() * last_frm_cam_pose_wc;
    }
    else {
        twist_is_valid_ = false;
        twist_ = Mat44_t::Identity();
    }
}

void tracking_module::replace_landmarks_in_last_frm(nondeterministic::unordered_map<std::shared_ptr<data::landmark>, std::shared_ptr<data::landmark>>& replaced_lms) {
    std::lock_guard<std::mutex> lock(mtx_last_frm_);
    for (unsigned int idx = 0; idx < last_frm_.frm_obs_.num_keypts_; ++idx) {
        const auto& lm = last_frm_.get_landmark(idx);
        if (!lm) {
            continue;
        }

        if (replaced_lms.count(lm)) {
            auto replaced_lm = replaced_lms[lm];
            if (last_frm_.has_landmark(replaced_lm)) {
                last_frm_.erase_landmark(replaced_lm);
            }
            last_frm_.add_landmark(replaced_lm, idx);
        }
    }
}

void tracking_module::update_last_frame() {
    auto last_ref_keyfrm = last_frm_.ref_keyfrm_;
    if (!last_ref_keyfrm) {
        return;
    }
    last_frm_.set_pose_cw(last_cam_pose_from_ref_keyfrm_ * last_ref_keyfrm->get_pose_cw());
}

bool tracking_module::optimize_current_frame_with_local_map(unsigned int& num_tracked_lms,
                                                            unsigned int& num_reliable_lms,
                                                            const unsigned int min_num_obs_thr) {
    // optimize the pose
    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    pose_optimizer_->optimize(curr_frm_, optimized_pose, outlier_flags);
    curr_frm_.set_pose_cw(optimized_pose);

    // Reject outliers
    for (unsigned int idx = 0; idx < curr_frm_.frm_obs_.num_keypts_; ++idx) {
        if (!outlier_flags.at(idx)) {
            continue;
        }
        curr_frm_.erase_landmark_with_index(idx);
    }

    // count up the number of tracked landmarks
    num_tracked_lms = 0;
    num_reliable_lms = 0;
    for (unsigned int idx = 0; idx < curr_frm_.frm_obs_.num_keypts_; ++idx) {
        const auto& lm = curr_frm_.get_landmark(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // the observation has been considered as inlier in the pose optimization
        assert(lm->has_observation());
        // count up
        if (0 < min_num_obs_thr) {
            if (min_num_obs_thr <= lm->num_observations()) {
                ++num_reliable_lms;
            }
        }
        ++num_tracked_lms;
        // increment the number of tracked frame
        lm->increase_num_observed();
    }

    constexpr unsigned int num_tracked_lms_thr = 20;

    // if recently relocalized, use the more strict threshold
    if (curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0 && num_tracked_lms < 2 * num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms, 2 * num_tracked_lms_thr);
        return false;
    }

    // check the threshold of the number of tracked landmarks
    if (num_tracked_lms < num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms, num_tracked_lms_thr);
        return false;
    }

    return true;
}

bool tracking_module::update_local_map(unsigned int fixed_keyframe_id_threshold) {
    // clean landmark associations
    for (unsigned int idx = 0; idx < curr_frm_.frm_obs_.num_keypts_; ++idx) {
        const auto& lm = curr_frm_.get_landmark(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            curr_frm_.erase_landmark_with_index(idx);
            continue;
        }
    }

    // acquire the current local map
    local_landmarks_.clear();
    auto local_map_updater = module::local_map_updater(max_num_local_keyfrms_);
    if (!local_map_updater.acquire_local_map(curr_frm_.get_landmarks(), curr_frm_.frm_obs_.num_keypts_, fixed_keyframe_id_threshold)) {
        return false;
    }
    // update the variables
    local_landmarks_ = local_map_updater.get_local_landmarks();
    auto nearest_covisibility = local_map_updater.get_nearest_covisibility();

    // update the reference keyframe for the current frame
    if (nearest_covisibility) {
        curr_frm_.ref_keyfrm_ = nearest_covisibility;
    }

    map_db_->set_local_landmarks(local_landmarks_);
    return true;
}

bool tracking_module::search_local_landmarks() {
    // select the landmarks which can be reprojected from the ones observed in the current frame
    std::unordered_set<unsigned int> curr_landmark_ids;
    for (const auto& lm : curr_frm_.get_landmarks()) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // this landmark cannot be reprojected
        // because already observed in the current frame
        curr_landmark_ids.insert(lm->id_);

        // this landmark is observable from the current frame
        lm->increase_num_observable();
    }

    bool found_proj_candidate = false;
    // temporary variables
    Vec2_t reproj;
    float x_right;
    unsigned int pred_scale_level;
    eigen_alloc_unord_map<unsigned int, Vec2_t> lm_to_reproj;
    std::unordered_map<unsigned int, float> lm_to_x_right;
    std::unordered_map<unsigned int, int> lm_to_scale;
    for (const auto& lm : local_landmarks_) {
        if (curr_landmark_ids.count(lm->id_)) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // check the observability
        if (curr_frm_.can_observe(lm, 0.5, reproj, x_right, pred_scale_level)) {
            lm_to_reproj[lm->id_] = reproj;
            lm_to_x_right[lm->id_] = x_right;
            lm_to_scale[lm->id_] = pred_scale_level;

            // this landmark is observable from the current frame
            lm->increase_num_observable();

            found_proj_candidate = true;
        }
    }

    if (!found_proj_candidate) {
        spdlog::warn("projection candidate not found");
        return false;
    }

    // acquire more 2D-3D matches by projecting the local landmarks to the current frame
    match::projection projection_matcher(0.8);
    const float margin = (curr_frm_.id_ < last_reloc_frm_id_ + 2)
                             ? 20.0
                             : ((camera_->setup_type_ == camera::setup_type_t::RGBD)
                                    ? 10.0
                                    : 5.0);
    projection_matcher.match_frame_and_landmarks(curr_frm_, local_landmarks_, lm_to_reproj, lm_to_x_right, lm_to_scale, margin);
    return true;
}

bool tracking_module::new_keyframe_is_needed(unsigned int num_tracked_lms,
                                             unsigned int num_reliable_lms,
                                             const unsigned int min_num_obs_thr) const {
    // cannnot insert the new keyframe in a second after relocalization
    if (curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0) {
        return false;
    }

    // check the new keyframe is needed
    return keyfrm_inserter_.new_keyframe_is_needed(map_db_, curr_frm_, num_tracked_lms, num_reliable_lms, *curr_frm_.ref_keyfrm_, min_num_obs_thr);
}

std::future<void> tracking_module::async_stop_keyframe_insertion() {
    auto future_stop_keyframe_insertion = std::async(
        std::launch::async,
        [this]() {
            std::lock_guard<std::mutex> lock(mtx_stop_keyframe_insertion_);
            SPDLOG_TRACE("tracking_module: stop keyframe insertion");
            is_stopped_keyframe_insertion_ = true;
        });
    return future_stop_keyframe_insertion;
}

std::future<void> tracking_module::async_start_keyframe_insertion() {
    auto future_stop_keyframe_insertion = std::async(
        std::launch::async,
        [this]() {
            std::lock_guard<std::mutex> lock(mtx_stop_keyframe_insertion_);
            SPDLOG_TRACE("tracking_module: start keyframe insertion");
            is_stopped_keyframe_insertion_ = false;
        });
    return future_stop_keyframe_insertion;
}

std::shared_future<void> tracking_module::async_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    pause_is_requested_ = true;
    if (!future_pause_.valid()) {
        future_pause_ = promise_pause_.get_future().share();
    }

    std::shared_future<void> future_pause = future_pause_;
    if (is_paused_) {
        promise_pause_.set_value();
        // Clear request
        promise_pause_ = std::promise<void>();
        future_pause_ = std::shared_future<void>();
    }
    return future_pause;
}

bool tracking_module::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool tracking_module::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

void tracking_module::resume() {
    std::lock_guard<std::mutex> lock(mtx_pause_);

    is_paused_ = false;
    pause_is_requested_ = false;

    spdlog::info("resume tracking module");
}

bool tracking_module::pause_if_requested() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (pause_is_requested_) {
        is_paused_ = true;
        spdlog::info("pause tracking module");
        promise_pause_.set_value();
        promise_pause_ = std::promise<void>();
        future_pause_ = std::shared_future<void>();
        return true;
    }
    else {
        return false;
    }
}

} // namespace stella_vslam
