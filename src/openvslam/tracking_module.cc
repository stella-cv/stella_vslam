#include "openvslam/config.h"
#include "openvslam/system.h"
#include "openvslam/tracking_module.h"
#include "openvslam/mapping_module.h"
#include "openvslam/global_optimization_module.h"
#include "openvslam/camera/base.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/feature/orb_extractor.h"
#include "openvslam/match/projection.h"
#include "openvslam/module/local_map_updater.h"
#include "openvslam/util/image_converter.h"
#include "openvslam/util/yaml.h"

#include <chrono>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace {
using namespace openvslam;

feature::orb_params get_orb_params(const YAML::Node& yaml_node) {
    spdlog::debug("load ORB parameters");
    try {
        return feature::orb_params(yaml_node);
    }
    catch (const std::exception& e) {
        spdlog::error("failed in loading ORB parameters: {}", e.what());
        throw;
    }
}

double get_true_depth_thr(const camera::base* camera, const YAML::Node& yaml_node) {
    spdlog::debug("load depth threshold");
    double true_depth_thr = 40.0;
    if (camera->setup_type_ == camera::setup_type_t::Stereo || camera->setup_type_ == camera::setup_type_t::RGBD) {
        const auto depth_thr_factor = yaml_node["depth_threshold"].as<double>(40.0);
        true_depth_thr = camera->true_baseline_ * depth_thr_factor;
    }
    return true_depth_thr;
}

double get_depthmap_factor(const camera::base* camera, const YAML::Node& yaml_node) {
    spdlog::debug("load depthmap factor");
    double depthmap_factor = 1.0;
    if (camera->setup_type_ == camera::setup_type_t::RGBD) {
        depthmap_factor = yaml_node["depthmap_factor"].as<double>(depthmap_factor);
    }
    if (depthmap_factor < 0.) {
        throw std::runtime_error("depthmap_factor must be greater than 0");
    }
    return depthmap_factor;
}

double get_reloc_distance_threshold(const YAML::Node& yaml_node) {
    spdlog::debug("load maximum distance threshold where close keyframes could be found");
    return yaml_node["reloc_distance_threshold"].as<double>(0.2);
}

double get_reloc_angle_threshold(const YAML::Node& yaml_node) {
    spdlog::debug("load maximum angle threshold between given pose and close keyframes");
    return yaml_node["reloc_angle_threshold"].as<double>(0.45);
}

} // unnamed namespace

namespace openvslam {

tracking_module::tracking_module(const std::shared_ptr<config>& cfg, system* system, data::map_database* map_db,
                                 data::bow_vocabulary* bow_vocab, data::bow_database* bow_db)
    : camera_(cfg->camera_), true_depth_thr_(get_true_depth_thr(camera_, cfg->yaml_node_)),
      depthmap_factor_(get_depthmap_factor(camera_, cfg->yaml_node_)),
      reloc_distance_threshold_(get_reloc_distance_threshold(cfg->yaml_node_)),
      reloc_angle_threshold_(get_reloc_angle_threshold(cfg->yaml_node_)),
      system_(system), map_db_(map_db), bow_vocab_(bow_vocab), bow_db_(bow_db),
      initializer_(cfg->camera_->setup_type_, map_db, bow_db, util::yaml_optional_ref(cfg->yaml_node_, "Initializer")),
      frame_tracker_(camera_, 10), relocalizer_(bow_db_), pose_optimizer_(),
      keyfrm_inserter_(cfg->camera_->setup_type_, true_depth_thr_, map_db, bow_db, 0, cfg->camera_->fps_) {
    spdlog::debug("CONSTRUCT: tracking_module");

    feature::orb_params orb_params = get_orb_params(util::yaml_optional_ref(cfg->yaml_node_, "Feature"));
    extractor_left_ = new feature::orb_extractor(orb_params);
    if (camera_->setup_type_ == camera::setup_type_t::Monocular) {
        ini_extractor_left_ = new feature::orb_extractor(orb_params);
        ini_extractor_left_->set_max_num_keypoints(orb_params.ini_max_num_keypts_);
    }
    if (camera_->setup_type_ == camera::setup_type_t::Stereo) {
        extractor_right_ = new feature::orb_extractor(orb_params);
    }
}

tracking_module::~tracking_module() {
    delete extractor_left_;
    extractor_left_ = nullptr;
    delete extractor_right_;
    extractor_right_ = nullptr;
    delete ini_extractor_left_;
    ini_extractor_left_ = nullptr;

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

std::shared_ptr<Mat44_t> tracking_module::track_monocular_image(const cv::Mat& img, const double timestamp, const cv::Mat& mask) {
    const auto start = std::chrono::system_clock::now();

    // color conversion
    img_gray_ = img;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);

    // create current frame object
    if (tracking_state_ == tracker_state_t::NotInitialized || tracking_state_ == tracker_state_t::Initializing) {
        curr_frm_ = data::frame(img_gray_, timestamp, ini_extractor_left_, bow_vocab_, camera_, true_depth_thr_, mask);
    }
    else {
        curr_frm_ = data::frame(img_gray_, timestamp, extractor_left_, bow_vocab_, camera_, true_depth_thr_, mask);
    }

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::shared_ptr<Mat44_t> cam_pose_wc = nullptr;
    if (curr_frm_.cam_pose_cw_is_valid_) {
        cam_pose_wc = std::allocate_shared<Mat44_t>(Eigen::aligned_allocator<Mat44_t>(), curr_frm_.get_cam_pose_inv());
    }
    return cam_pose_wc;
}

std::shared_ptr<Mat44_t> tracking_module::track_stereo_image(const cv::Mat& left_img_rect, const cv::Mat& right_img_rect, const double timestamp, const cv::Mat& mask) {
    const auto start = std::chrono::system_clock::now();

    // color conversion
    img_gray_ = left_img_rect;
    cv::Mat right_img_gray = right_img_rect;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);
    util::convert_to_grayscale(right_img_gray, camera_->color_order_);

    // create current frame object
    curr_frm_ = data::frame(img_gray_, right_img_gray, timestamp, extractor_left_, extractor_right_, bow_vocab_, camera_, true_depth_thr_, mask);

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::shared_ptr<Mat44_t> cam_pose_wc = nullptr;
    if (curr_frm_.cam_pose_cw_is_valid_) {
        cam_pose_wc = std::allocate_shared<Mat44_t>(Eigen::aligned_allocator<Mat44_t>(), curr_frm_.get_cam_pose_inv());
    }
    return cam_pose_wc;
}

std::shared_ptr<Mat44_t> tracking_module::track_RGBD_image(const cv::Mat& img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask) {
    const auto start = std::chrono::system_clock::now();

    // color and depth scale conversion
    img_gray_ = img;
    cv::Mat img_depth = depthmap;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);
    util::convert_to_true_depth(img_depth, depthmap_factor_);

    // create current frame object
    curr_frm_ = data::frame(img_gray_, img_depth, timestamp, extractor_left_, bow_vocab_, camera_, true_depth_thr_, mask);

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::shared_ptr<Mat44_t> cam_pose_wc = nullptr;
    if (curr_frm_.cam_pose_cw_is_valid_) {
        cam_pose_wc = std::allocate_shared<Mat44_t>(Eigen::aligned_allocator<Mat44_t>(), curr_frm_.get_cam_pose_inv());
    }
    return cam_pose_wc;
}

bool tracking_module::request_update_pose(const Mat44_t& pose) {
    std::lock_guard<std::mutex> lock(mtx_update_pose_request_);
    if (update_pose_is_requested_) {
        spdlog::warn("Can not process new pose update request while previous was not finished");
        return false;
    }
    update_pose_is_requested_ = true;
    requested_pose_ = pose;
    return true;
}

bool tracking_module::update_pose_is_requested() {
    std::lock_guard<std::mutex> lock(mtx_update_pose_request_);
    return update_pose_is_requested_;
}

Mat44_t& tracking_module::get_requested_pose() {
    std::lock_guard<std::mutex> lock(mtx_update_pose_request_);
    return requested_pose_;
}

void tracking_module::finish_update_pose_request() {
    std::lock_guard<std::mutex> lock(mtx_update_pose_request_);
    update_pose_is_requested_ = false;
}

void tracking_module::reset() {
    spdlog::info("resetting system");

    initializer_.reset();
    keyfrm_inserter_.reset();

    mapper_->request_reset();
    global_optimizer_->request_reset();

    bow_db_->clear();
    map_db_->clear();

    data::frame::next_id_ = 0;
    data::keyframe::next_id_ = 0;
    data::landmark::next_id_ = 0;

    last_reloc_frm_id_ = 0;

    tracking_state_ = tracker_state_t::NotInitialized;
}

void tracking_module::track() {
    if (tracking_state_ == tracker_state_t::NotInitialized) {
        tracking_state_ = tracker_state_t::Initializing;
    }

    last_tracking_state_ = tracking_state_;

    // check if pause is requested
    check_and_execute_pause();
    while (is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // LOCK the map database
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    if (tracking_state_ == tracker_state_t::Initializing) {
        if (!initialize()) {
            return;
        }

        // update the reference keyframe, local keyframes, and local landmarks
        update_local_map();

        // pass all of the keyframes to the mapping module
        const auto keyfrms = map_db_->get_all_keyframes();
        for (const auto keyfrm : keyfrms) {
            mapper_->queue_keyframe(keyfrm);
        }

        // state transition to Tracking mode
        tracking_state_ = tracker_state_t::Tracking;
    }
    else {
        // apply replace of landmarks observed in the last frame
        apply_landmark_replace();
        // update the camera pose of the last frame
        // because the mapping module might optimize the camera pose of the last frame's reference keyframe
        update_last_frame();

        // set the reference keyframe of the current frame
        curr_frm_.ref_keyfrm_ = last_frm_.ref_keyfrm_;

        auto succeeded = track_current_frame();

        // update the local map and optimize the camera pose of the current frame
        if (succeeded) {
            update_local_map();
            succeeded = optimize_current_frame_with_local_map();
        }

        // update the motion model
        if (succeeded) {
            update_motion_model();
        }

        // state transition
        tracking_state_ = succeeded ? tracker_state_t::Tracking : tracker_state_t::Lost;

        // update the frame statistics
        map_db_->update_frame_statistics(curr_frm_, tracking_state_ == tracker_state_t::Lost);

        // if tracking is failed within 5.0 sec after initialization, reset the system
        constexpr float init_retry_thr = 5.0;
        if (tracking_state_ == tracker_state_t::Lost
            && curr_frm_.id_ - initializer_.get_initial_frame_id() < camera_->fps_ * init_retry_thr) {
            spdlog::info("tracking lost within {} sec after initialization", init_retry_thr);
            system_->request_reset();
            return;
        }

        // show message if tracking has been lost
        if (last_tracking_state_ != tracker_state_t::Lost && tracking_state_ == tracker_state_t::Lost) {
            spdlog::info("tracking lost: frame {}", curr_frm_.id_);
        }

        // check to insert the new keyframe derived from the current frame
        if (succeeded && new_keyframe_is_needed()) {
            insert_new_keyframe();
        }

        // tidy up observations
        for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
            if (curr_frm_.landmarks_.at(idx) && curr_frm_.outlier_flags_.at(idx)) {
                curr_frm_.landmarks_.at(idx) = nullptr;
            }
        }
    }

    // store the relative pose from the reference keyframe to the current frame
    // to update the camera pose at the beginning of the next tracking process
    if (curr_frm_.cam_pose_cw_is_valid_) {
        last_cam_pose_from_ref_keyfrm_ = curr_frm_.cam_pose_cw_ * curr_frm_.ref_keyfrm_->get_cam_pose_inv();
    }

    // update last frame
    last_frm_ = curr_frm_;
}

bool tracking_module::initialize() {
    // try to initialize with the current frame
    initializer_.initialize(curr_frm_);

    // if map building was failed -> reset the map database
    if (initializer_.get_state() == module::initializer_state_t::Wrong) {
        // reset
        system_->request_reset();
        return false;
    }

    // if initializing was failed -> try to initialize with the next frame
    if (initializer_.get_state() != module::initializer_state_t::Succeeded) {
        return false;
    }

    // succeeded
    return true;
}

bool tracking_module::track_current_frame() {
    bool succeeded = false;

    if (update_pose_is_requested()) {
        // Force relocalization by pose
        curr_frm_.set_cam_pose(get_requested_pose());

        curr_frm_.compute_bow();
        const auto candidates = map_db_->get_close_keyframes(get_requested_pose(),
                                                             reloc_distance_threshold_,
                                                             reloc_angle_threshold_);
        if (!candidates.empty()) {
            succeeded = relocalizer_.reloc_by_candidates(curr_frm_, candidates);
            if (succeeded) {
                last_reloc_frm_id_ = curr_frm_.id_;
            }
        }
        else {
            curr_frm_.cam_pose_cw_is_valid_ = false;
        }
        finish_update_pose_request();
        return succeeded;
    }

    if (tracking_state_ == tracker_state_t::Tracking) {
        // Tracking mode
        if (twist_is_valid_ && last_reloc_frm_id_ + 2 < curr_frm_.id_) {
            // if the motion model is valid
            succeeded = frame_tracker_.motion_based_track(curr_frm_, last_frm_, twist_);
        }
        if (!succeeded) {
            succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, curr_frm_.ref_keyfrm_);
        }
        if (!succeeded) {
            succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, curr_frm_.ref_keyfrm_);
        }
    }
    else {
        // Lost mode
        // try to relocalize
        succeeded = relocalizer_.relocalize(curr_frm_);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
        }
    }

    return succeeded;
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
    for (unsigned int idx = 0; idx < last_frm_.num_keypts_; ++idx) {
        auto lm = last_frm_.landmarks_.at(idx);
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

bool tracking_module::optimize_current_frame_with_local_map() {
    // acquire more 2D-3D matches by reprojecting the local landmarks to the current frame
    search_local_landmarks();

    // optimize the pose
    pose_optimizer_.optimize(curr_frm_);

    // count up the number of tracked landmarks
    num_tracked_lms_ = 0;
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        if (!curr_frm_.outlier_flags_.at(idx)) {
            // the observation has been considered as inlier in the pose optimization
            assert(lm->has_observation());
            // count up
            ++num_tracked_lms_;
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
    if (curr_frm_.id_ < last_reloc_frm_id_ + camera_->fps_ && num_tracked_lms_ < 2 * num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms_, 2 * num_tracked_lms_thr);
        return false;
    }

    // check the threshold of the number of tracked landmarks
    if (num_tracked_lms_ < num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms_, num_tracked_lms_thr);
        return false;
    }

    return true;
}

void tracking_module::update_local_map() {
    // clean landmark associations
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
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
    if (!local_map_updater.acquire_local_map()) {
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

void tracking_module::search_local_landmarks() {
    // select the landmarks which can be reprojected from the ones observed in the current frame
    for (auto lm : curr_frm_.landmarks_) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // this landmark cannot be reprojected
        // because already observed in the current frame
        lm->is_observable_in_tracking_ = false;
        lm->identifier_in_local_lm_search_ = curr_frm_.id_;

        // this landmark is observable from the current frame
        lm->increase_num_observable();
    }

    bool found_proj_candidate = false;
    // temporary variables
    Vec2_t reproj;
    float x_right;
    unsigned int pred_scale_level;
    for (auto lm : local_landmarks_) {
        // avoid the landmarks which cannot be reprojected (== observed in the current frame)
        if (lm->identifier_in_local_lm_search_ == curr_frm_.id_) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // check the observability
        if (curr_frm_.can_observe(lm, 0.5, reproj, x_right, pred_scale_level)) {
            // pass the temporary variables
            lm->reproj_in_tracking_ = reproj;
            lm->x_right_in_tracking_ = x_right;
            lm->scale_level_in_tracking_ = pred_scale_level;

            // this landmark can be reprojected
            lm->is_observable_in_tracking_ = true;

            // this landmark is observable from the current frame
            lm->increase_num_observable();

            found_proj_candidate = true;
        }
        else {
            // this landmark cannot be reprojected
            lm->is_observable_in_tracking_ = false;
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
    projection_matcher.match_frame_and_landmarks(curr_frm_, local_landmarks_, margin);
}

bool tracking_module::new_keyframe_is_needed() const {
    if (!mapping_is_enabled_) {
        return false;
    }

    // cannnot insert the new keyframe in a second after relocalization
    const auto num_keyfrms = map_db_->get_num_keyframes();
    if (camera_->fps_ < num_keyfrms && curr_frm_.id_ < last_reloc_frm_id_ + camera_->fps_) {
        return false;
    }

    // check the new keyframe is needed
    return keyfrm_inserter_.new_keyframe_is_needed(curr_frm_, num_tracked_lms_, *curr_frm_.ref_keyfrm_);
}

void tracking_module::insert_new_keyframe() {
    // insert the new keyframe
    const auto ref_keyfrm = keyfrm_inserter_.insert_new_keyframe(curr_frm_);
    // set the reference keyframe with the new keyframe
    if (ref_keyfrm) {
        curr_frm_.ref_keyfrm_ = ref_keyfrm;
    }
}

void tracking_module::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
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
        return true;
    }
    else {
        return false;
    }
}

} // namespace openvslam
