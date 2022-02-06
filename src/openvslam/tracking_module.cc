#include "openvslam/config.h"
#include "openvslam/system.h"
#include "openvslam/tracking_module.h"
#include "openvslam/mapping_module.h"
#include "openvslam/global_optimization_module.h"
#include "openvslam/camera/base.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/match/projection.h"
#include "openvslam/module/local_map_updater.h"
#include "openvslam/util/yaml.h"

#include <chrono>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace {
using namespace openvslam;

double get_reloc_distance_threshold(const YAML::Node& yaml_node) {
    spdlog::debug("load maximum distance threshold where close keyframes could be found");
    return yaml_node["reloc_distance_threshold"].as<double>(0.2);
}

double get_reloc_angle_threshold(const YAML::Node& yaml_node) {
    spdlog::debug("load maximum angle threshold between given pose and close keyframes");
    return yaml_node["reloc_angle_threshold"].as<double>(0.45);
}

double get_enable_auto_relocalization(const YAML::Node& yaml_node) {
    return yaml_node["enable_auto_relocalization"].as<bool>(true);
}

double get_use_robust_matcher_for_relocalization_request(const YAML::Node& yaml_node) {
    return yaml_node["use_robust_matcher_for_relocalization_request"].as<bool>(false);
}

} // unnamed namespace

namespace openvslam {

tracking_module::tracking_module(const std::shared_ptr<config>& cfg, data::map_database* map_db,
                                 data::bow_vocabulary* bow_vocab, data::bow_database* bow_db)
    : camera_(cfg->camera_),
      reloc_distance_threshold_(get_reloc_distance_threshold(util::yaml_optional_ref(cfg->yaml_node_, "Tracking"))),
      reloc_angle_threshold_(get_reloc_angle_threshold(util::yaml_optional_ref(cfg->yaml_node_, "Tracking"))),
      enable_auto_relocalization_(get_enable_auto_relocalization(util::yaml_optional_ref(cfg->yaml_node_, "Tracking"))),
      use_robust_matcher_for_relocalization_request_(get_use_robust_matcher_for_relocalization_request(util::yaml_optional_ref(cfg->yaml_node_, "Tracking"))),
      map_db_(map_db), bow_vocab_(bow_vocab), bow_db_(bow_db),
      initializer_(map_db, bow_db, util::yaml_optional_ref(cfg->yaml_node_, "Initializer")),
      frame_tracker_(camera_, 10),
      relocalizer_(util::yaml_optional_ref(cfg->yaml_node_, "Relocalizer")),
      pose_optimizer_(),
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

void tracking_module::set_mapping_module_status(const bool mapping_is_enabled) {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    mapping_is_enabled_ = mapping_is_enabled;
}

bool tracking_module::get_mapping_module_status() const {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    return mapping_is_enabled_;
}

std::vector<cv::KeyPoint> tracking_module::get_initial_keypoints() const {
    return initializer_.get_initial_keypoints();
}

std::vector<int> tracking_module::get_initial_matches() const {
    return initializer_.get_initial_matches();
}

bool tracking_module::request_relocalize_by_pose(const Mat44_t& pose) {
    std::lock_guard<std::mutex> lock(mtx_relocalize_by_pose_request_);
    if (relocalize_by_pose_is_requested_) {
        spdlog::warn("Can not process new pose update request while previous was not finished");
        return false;
    }
    relocalize_by_pose_is_requested_ = true;
    relocalize_by_pose_request_.mode_2d_ = false;
    relocalize_by_pose_request_.pose_ = pose;
    return true;
}

bool tracking_module::request_relocalize_by_pose_2d(const Mat44_t& pose, const Vec3_t& normal_vector) {
    std::lock_guard<std::mutex> lock(mtx_relocalize_by_pose_request_);
    if (relocalize_by_pose_is_requested_) {
        spdlog::warn("Can not process new pose update request while previous was not finished");
        return false;
    }
    relocalize_by_pose_is_requested_ = true;
    relocalize_by_pose_request_.mode_2d_ = true;
    relocalize_by_pose_request_.pose_ = pose;
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
    data::keyframe::next_id_ = 0;
    data::landmark::next_id_ = 0;

    last_reloc_frm_id_ = 0;
    last_reloc_frm_timestamp_ = 0.0;

    tracking_state_ = tracker_state_t::Initializing;
}

std::shared_ptr<Mat44_t> tracking_module::feed_frame(data::frame curr_frm) {
    // check if pause is requested
    check_and_execute_pause();
    while (is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    curr_frm_ = curr_frm;

    bool succeeded = false;
    if (tracking_state_ == tracker_state_t::Initializing) {
        succeeded = initialize();
    }
    else {
        bool relocalization_is_needed = tracking_state_ == tracker_state_t::Lost;
        succeeded = track(relocalization_is_needed);
    }

    // state transition
    if (succeeded) {
        tracking_state_ = tracker_state_t::Tracking;
    }
    else if (tracking_state_ == tracker_state_t::Tracking) {
        tracking_state_ = tracker_state_t::Lost;

        spdlog::info("tracking lost: frame {}", curr_frm_.id_);
        // if tracking is failed within 5.0 sec after initialization, reset the system
        constexpr float init_retry_thr = 5.0;
        if (curr_frm_.timestamp_ - initializer_.get_initial_frame_timestamp() < init_retry_thr) {
            spdlog::info("tracking lost within {} sec after initialization", init_retry_thr);
            reset();
            return nullptr;
        }
    }

    std::shared_ptr<Mat44_t> cam_pose_wc = nullptr;
    // store the relative pose from the reference keyframe to the current frame
    // to update the camera pose at the beginning of the next tracking process
    if (curr_frm_.cam_pose_cw_is_valid_) {
        last_cam_pose_from_ref_keyfrm_ = curr_frm_.cam_pose_cw_ * curr_frm_.ref_keyfrm_->get_cam_pose_inv();
        cam_pose_wc = std::allocate_shared<Mat44_t>(Eigen::aligned_allocator<Mat44_t>(), curr_frm_.get_cam_pose_inv());
    }

    // update last frame
    last_frm_ = curr_frm_;

    return cam_pose_wc;
}

bool tracking_module::track(bool relocalization_is_needed) {
    // LOCK the map database
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // apply replace of landmarks observed in the last frame
    apply_landmark_replace();
    // update the camera pose of the last frame
    // because the mapping module might optimize the camera pose of the last frame's reference keyframe
    update_last_frame();

    // set the reference keyframe of the current frame
    curr_frm_.ref_keyfrm_ = last_frm_.ref_keyfrm_;

    bool succeeded = false;
    std::unordered_set<unsigned int> outlier_ids;
    if (relocalize_by_pose_is_requested()) {
        // Force relocalization by pose
        succeeded = relocalize_by_pose(get_relocalize_by_pose_request());
    }
    else if (!relocalization_is_needed) {
        succeeded = track_current_frame(outlier_ids);
    }
    else if (enable_auto_relocalization_) {
        // Compute the BoW representations to perform relocalization
        if (!curr_frm_.bow_is_available()) {
            curr_frm_.compute_bow(bow_vocab_);
        }
        // try to relocalize
        succeeded = relocalizer_.relocalize(bow_db_, curr_frm_);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
            last_reloc_frm_timestamp_ = curr_frm_.timestamp_;
        }
    }

    // update the local map and optimize the camera pose of the current frame
    unsigned int num_tracked_lms = 0;
    if (succeeded) {
        update_local_map(outlier_ids);
        succeeded = optimize_current_frame_with_local_map(num_tracked_lms, outlier_ids);
    }

    // update the motion model
    if (succeeded) {
        update_motion_model();
    }

    // check to insert the new keyframe derived from the current frame
    if (succeeded && new_keyframe_is_needed(num_tracked_lms)) {
        insert_new_keyframe();
    }

    // tidy up observations
    for (unsigned int idx = 0; idx < curr_frm_.frm_obs_.num_keypts_; ++idx) {
        if (curr_frm_.landmarks_.at(idx) && curr_frm_.outlier_flags_.at(idx)) {
            curr_frm_.landmarks_.at(idx) = nullptr;
        }
    }

    // update the frame statistics
    map_db_->update_frame_statistics(curr_frm_, !succeeded);

    return succeeded;
}

bool tracking_module::initialize() {
    // LOCK the map database
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // try to initialize with the current frame
    initializer_.initialize(camera_->setup_type_, bow_vocab_, curr_frm_);

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
    const auto keyfrms = map_db_->get_all_keyframes();
    for (const auto& keyfrm : keyfrms) {
        mapper_->queue_keyframe(keyfrm);
    }

    // succeeded
    return true;
}

bool tracking_module::track_current_frame(std::unordered_set<unsigned int>& outlier_ids) {
    bool succeeded = false;

    // Tracking mode
    if (twist_is_valid_ && last_reloc_frm_id_ + 2 < curr_frm_.id_) {
        // if the motion model is valid
        succeeded = frame_tracker_.motion_based_track(curr_frm_, last_frm_, twist_, outlier_ids);
    }
    if (!succeeded) {
        // Compute the BoW representations to perform the BoW match
        if (!curr_frm_.bow_is_available()) {
            curr_frm_.compute_bow(bow_vocab_);
        }
        succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, curr_frm_.ref_keyfrm_, outlier_ids);
    }
    if (!succeeded) {
        succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, curr_frm_.ref_keyfrm_, outlier_ids);
    }

    return succeeded;
}

bool tracking_module::relocalize_by_pose(const pose_request& request) {
    bool succeeded = false;
    curr_frm_.set_cam_pose(request.pose_);

    if (!curr_frm_.bow_is_available()) {
        curr_frm_.compute_bow(bow_vocab_);
    }
    const auto candidates = get_close_keyframes(request);

    if (!candidates.empty()) {
        succeeded = relocalizer_.reloc_by_candidates(curr_frm_, candidates, use_robust_matcher_for_relocalization_request_);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
            last_reloc_frm_timestamp_ = curr_frm_.timestamp_;
        }
    }
    else {
        curr_frm_.cam_pose_cw_is_valid_ = false;
    }
    finish_relocalize_by_pose_request();
    return succeeded;
}

std::vector<std::shared_ptr<data::keyframe>> tracking_module::get_close_keyframes(const pose_request& request) {
    if (request.mode_2d_) {
        return map_db_->get_close_keyframes_2d(
            request.pose_,
            request.normal_vector_,
            reloc_distance_threshold_,
            reloc_angle_threshold_);
    }
    else {
        return map_db_->get_close_keyframes(
            request.pose_,
            reloc_distance_threshold_,
            reloc_angle_threshold_);
    }
}

void tracking_module::update_motion_model() {
    if (last_frm_.cam_pose_cw_is_valid_) {
        Mat44_t last_frm_cam_pose_wc = Mat44_t::Identity();
        last_frm_cam_pose_wc.block<3, 3>(0, 0) = last_frm_.get_rotation_inv();
        last_frm_cam_pose_wc.block<3, 1>(0, 3) = last_frm_.get_cam_center();
        twist_is_valid_ = true;
        twist_ = curr_frm_.cam_pose_cw_ * last_frm_cam_pose_wc;
    }
    else {
        twist_is_valid_ = false;
        twist_ = Mat44_t::Identity();
    }
}

void tracking_module::apply_landmark_replace() {
    for (unsigned int idx = 0; idx < last_frm_.frm_obs_.num_keypts_; ++idx) {
        auto& lm = last_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        auto replaced_lm = lm->get_replaced();
        if (replaced_lm) {
            last_frm_.landmarks_.at(idx) = replaced_lm;
        }
    }
}

void tracking_module::update_last_frame() {
    auto last_ref_keyfrm = last_frm_.ref_keyfrm_;
    if (!last_ref_keyfrm) {
        return;
    }
    last_frm_.set_cam_pose(last_cam_pose_from_ref_keyfrm_ * last_ref_keyfrm->get_cam_pose());
}

bool tracking_module::optimize_current_frame_with_local_map(unsigned int& num_tracked_lms,
                                                            std::unordered_set<unsigned int>& outlier_ids) {
    // acquire more 2D-3D matches by reprojecting the local landmarks to the current frame
    search_local_landmarks(outlier_ids);

    // optimize the pose
    pose_optimizer_.optimize(curr_frm_);

    // count up the number of tracked landmarks
    num_tracked_lms = 0;
    for (unsigned int idx = 0; idx < curr_frm_.frm_obs_.num_keypts_; ++idx) {
        const auto& lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        if (!curr_frm_.outlier_flags_.at(idx)) {
            // the observation has been considered as inlier in the pose optimization
            assert(lm->has_observation());
            // count up
            ++num_tracked_lms;
            // increment the number of tracked frame
            lm->increase_num_observed();
        }
        else {
            // the observation has been considered as outlier in the pose optimization
            // remove the observation
            curr_frm_.landmarks_.at(idx) = nullptr;
        }
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

void tracking_module::update_local_map(std::unordered_set<unsigned int>& outlier_ids) {
    // clean landmark associations
    for (unsigned int idx = 0; idx < curr_frm_.frm_obs_.num_keypts_; ++idx) {
        const auto& lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            curr_frm_.landmarks_.at(idx) = nullptr;
            continue;
        }
    }

    // acquire the current local map
    constexpr unsigned int max_num_local_keyfrms = 60;
    auto local_map_updater = module::local_map_updater(curr_frm_, max_num_local_keyfrms);
    if (!local_map_updater.acquire_local_map(outlier_ids)) {
        return;
    }
    // update the variables
    local_keyfrms_ = local_map_updater.get_local_keyframes();
    local_landmarks_ = local_map_updater.get_local_landmarks();
    auto nearest_covisibility = local_map_updater.get_nearest_covisibility();

    // update the reference keyframe for the current frame
    if (nearest_covisibility) {
        curr_frm_.ref_keyfrm_ = nearest_covisibility;
    }

    map_db_->set_local_landmarks(local_landmarks_);
}

void tracking_module::search_local_landmarks(std::unordered_set<unsigned int>& outlier_ids) {
    // select the landmarks which can be reprojected from the ones observed in the current frame
    std::unordered_set<unsigned int> curr_landmark_ids;
    for (const auto& lm : curr_frm_.landmarks_) {
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
        if (curr_landmark_ids.count(curr_frm_.id_)) {
            continue;
        }
        if (outlier_ids.count(curr_frm_.id_)) {
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
        return;
    }

    // acquire more 2D-3D matches by projecting the local landmarks to the current frame
    match::projection projection_matcher(0.8);
    const float margin = (curr_frm_.id_ < last_reloc_frm_id_ + 2)
                             ? 20.0
                             : ((camera_->setup_type_ == camera::setup_type_t::RGBD)
                                    ? 10.0
                                    : 5.0);
    projection_matcher.match_frame_and_landmarks(curr_frm_, local_landmarks_, lm_to_reproj, lm_to_x_right, lm_to_scale, margin);
}

bool tracking_module::new_keyframe_is_needed(unsigned int num_tracked_lms) const {
    if (!mapping_is_enabled_) {
        return false;
    }

    // cannnot insert the new keyframe in a second after relocalization
    if (curr_frm_.timestamp_ < last_reloc_frm_timestamp_ + 1.0) {
        return false;
    }

    // check the new keyframe is needed
    return keyfrm_inserter_.new_keyframe_is_needed(map_db_, curr_frm_, num_tracked_lms, *curr_frm_.ref_keyfrm_);
}

void tracking_module::insert_new_keyframe() {
    // insert the new keyframe
    const auto ref_keyfrm = keyfrm_inserter_.insert_new_keyframe(map_db_, curr_frm_);
    // set the reference keyframe with the new keyframe
    if (ref_keyfrm) {
        curr_frm_.ref_keyfrm_ = ref_keyfrm;
    }
}

std::future<void> tracking_module::async_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
    promises_pause_.emplace_back();
    return promises_pause_.back().get_future();
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

bool tracking_module::check_and_execute_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (pause_is_requested_) {
        is_paused_ = true;
        spdlog::info("pause tracking module");
        for (auto& promise : promises_pause_) {
            promise.set_value();
        }
        promises_pause_.clear();
        return true;
    }
    else {
        return false;
    }
}

} // namespace openvslam
