#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/tracking_module.h"
#include "openvslam/mapping_module.h"
#include "openvslam/global_optimization_module.h"
#include "openvslam/camera/base.h"
#include "openvslam/data/camera_database.h"
#include "openvslam/data/common.h"
#include "openvslam/data/frame_observation.h"
#include "openvslam/data/orb_params_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/bow_vocabulary.h"
#include "openvslam/match/stereo.h"
#include "openvslam/feature/orb_extractor.h"
#include "openvslam/io/trajectory_io.h"
#include "openvslam/io/map_database_io.h"
#include "openvslam/publish/map_publisher.h"
#include "openvslam/publish/frame_publisher.h"
#include "openvslam/util/converter.h"
#include "openvslam/util/image_converter.h"
#include "openvslam/util/yaml.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace {
using namespace openvslam;

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
} // namespace

namespace openvslam {

system::system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path)
    : cfg_(cfg), camera_(cfg->camera_), orb_params_(cfg->orb_params_) {
    spdlog::debug("CONSTRUCT: system");

    std::ostringstream message_stream;

    message_stream << std::endl;
    message_stream << R"(  ___               __   _____ _      _   __  __ )" << std::endl;
    message_stream << R"( / _ \ _ __  ___ _ _\ \ / / __| |    /_\ |  \/  |)" << std::endl;
    message_stream << R"(| (_) | '_ \/ -_) ' \\ V /\__ \ |__ / _ \| |\/| |)" << std::endl;
    message_stream << R"( \___/| .__/\___|_||_|\_/ |___/____/_/ \_\_|  |_|)" << std::endl;
    message_stream << R"(      |_|                                        )" << std::endl;
    message_stream << std::endl;
    message_stream << "Copyright (C) 2019," << std::endl;
    message_stream << "National Institute of Advanced Industrial Science and Technology (AIST)" << std::endl;
    message_stream << "All rights reserved." << std::endl;
    message_stream << std::endl;
    message_stream << "This is free software," << std::endl;
    message_stream << "and you are welcome to redistribute it under certain conditions." << std::endl;
    message_stream << "See the LICENSE file." << std::endl;
    message_stream << std::endl;

    // show configuration
    message_stream << *cfg_ << std::endl;

    spdlog::info(message_stream.str());

    // load ORB vocabulary
    spdlog::info("loading ORB vocabulary: {}", vocab_file_path);
#ifdef USE_DBOW2
    bow_vocab_ = new data::bow_vocabulary();
    try {
        bow_vocab_->loadFromBinaryFile(vocab_file_path);
    }
    catch (const std::exception&) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab_;
        bow_vocab_ = nullptr;
        exit(EXIT_FAILURE);
    }
#else
    bow_vocab_ = new fbow::Vocabulary();
    bow_vocab_->readFromFile(vocab_file_path);
    if (!bow_vocab_->isValid()) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab_;
        bow_vocab_ = nullptr;
        exit(EXIT_FAILURE);
    }
#endif

    // database
    cam_db_ = new data::camera_database(camera_);
    map_db_ = new data::map_database();
    auto bow_database_yaml_node = util::yaml_optional_ref(cfg->yaml_node_, "BowDatabase");
    int reject_by_graph_distance = bow_database_yaml_node["reject_by_graph_distance"].as<bool>(false);
    int loop_min_distance_on_graph = bow_database_yaml_node["loop_min_distance_on_graph"].as<int>(30);
    bow_db_ = new data::bow_database(bow_vocab_, reject_by_graph_distance, loop_min_distance_on_graph);

    // frame and map publisher
    frame_publisher_ = std::shared_ptr<publish::frame_publisher>(new publish::frame_publisher(cfg_, map_db_));
    map_publisher_ = std::shared_ptr<publish::map_publisher>(new publish::map_publisher(cfg_, map_db_));

    // tracking module
    tracker_ = new tracking_module(cfg_, this, map_db_, bow_vocab_, bow_db_);
    // mapping module
    mapper_ = new mapping_module(cfg_->yaml_node_["Mapping"], map_db_, bow_db_, bow_vocab_);
    // global optimization module
    global_optimizer_ = new global_optimization_module(map_db_, bow_db_, bow_vocab_, cfg_->yaml_node_, camera_->setup_type_ != camera::setup_type_t::Monocular);

    // preprocessing modules
    const auto preprocessing_params = util::yaml_optional_ref(cfg->yaml_node_, "Preprocessing");
    depthmap_factor_ = get_depthmap_factor(camera_, preprocessing_params);
    auto mask_rectangles = preprocessing_params["mask_rectangles"].as<std::vector<std::vector<float>>>(std::vector<std::vector<float>>());
    for (const auto& v : mask_rectangles) {
        if (v.size() != 4) {
            throw std::runtime_error("mask rectangle must contain four parameters");
        }
        if (v.at(0) >= v.at(1)) {
            throw std::runtime_error("x_max must be greater than x_min");
        }
        if (v.at(2) >= v.at(3)) {
            throw std::runtime_error("y_max must be greater than x_min");
        }
    }

    orb_params_db_ = new data::orb_params_database(orb_params_);

    const auto max_num_keypoints = preprocessing_params["max_num_keypoints"].as<unsigned int>(2000);
    extractor_left_ = new feature::orb_extractor(orb_params_, max_num_keypoints, mask_rectangles);
    if (camera_->setup_type_ == camera::setup_type_t::Monocular) {
        const auto ini_max_num_keypoints = preprocessing_params["ini_max_num_keypoints"].as<unsigned int>(2 * extractor_left_->get_max_num_keypoints());
        ini_extractor_left_ = new feature::orb_extractor(orb_params_, ini_max_num_keypoints, mask_rectangles);
    }
    if (camera_->setup_type_ == camera::setup_type_t::Stereo) {
        extractor_right_ = new feature::orb_extractor(orb_params_, max_num_keypoints, mask_rectangles);
    }

    // connect modules each other
    tracker_->set_mapping_module(mapper_);
    tracker_->set_global_optimization_module(global_optimizer_);
    mapper_->set_tracking_module(tracker_);
    mapper_->set_global_optimization_module(global_optimizer_);
    global_optimizer_->set_tracking_module(tracker_);
    global_optimizer_->set_mapping_module(mapper_);
}

system::~system() {
    global_optimization_thread_.reset(nullptr);
    delete global_optimizer_;
    global_optimizer_ = nullptr;

    mapping_thread_.reset(nullptr);
    delete mapper_;
    mapper_ = nullptr;

    delete tracker_;
    tracker_ = nullptr;

    delete bow_db_;
    bow_db_ = nullptr;
    delete map_db_;
    map_db_ = nullptr;
    delete cam_db_;
    cam_db_ = nullptr;
    delete bow_vocab_;
    bow_vocab_ = nullptr;

    delete extractor_left_;
    extractor_left_ = nullptr;
    delete extractor_right_;
    extractor_right_ = nullptr;
    delete ini_extractor_left_;
    ini_extractor_left_ = nullptr;

    delete orb_params_db_;
    orb_params_db_ = nullptr;

    spdlog::debug("DESTRUCT: system");
}

void system::startup(const bool need_initialize) {
    spdlog::info("startup SLAM system");
    system_is_running_ = true;

    if (!need_initialize) {
        tracker_->tracking_state_ = tracker_state_t::Lost;
    }

    mapping_thread_ = std::unique_ptr<std::thread>(new std::thread(&openvslam::mapping_module::run, mapper_));
    global_optimization_thread_ = std::unique_ptr<std::thread>(new std::thread(&openvslam::global_optimization_module::run, global_optimizer_));
}

void system::shutdown() {
    // terminate the other threads
    auto future_mapper_terminate = mapper_->async_terminate();
    auto future_global_optimizer_terminate = global_optimizer_->async_terminate();
    future_mapper_terminate.get();
    future_global_optimizer_terminate.get();

    // wait until the threads stop
    mapping_thread_->join();
    global_optimization_thread_->join();

    spdlog::info("shutdown SLAM system");
    system_is_running_ = false;
}

void system::save_frame_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_frame_trajectory(path, format);
    resume_other_threads();
}

void system::save_keyframe_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_keyframe_trajectory(path, format);
    resume_other_threads();
}

void system::load_map_database(const std::string& path) const {
    pause_other_threads();
    io::map_database_io map_db_io(cam_db_, orb_params_db_, map_db_, bow_db_, bow_vocab_);
    map_db_io.load_message_pack(path);
    resume_other_threads();
}

void system::save_map_database(const std::string& path) const {
    pause_other_threads();
    io::map_database_io map_db_io(cam_db_, orb_params_db_, map_db_, bow_db_, bow_vocab_);
    map_db_io.save_message_pack(path);
    resume_other_threads();
}

const std::shared_ptr<publish::map_publisher> system::get_map_publisher() const {
    return map_publisher_;
}

const std::shared_ptr<publish::frame_publisher> system::get_frame_publisher() const {
    return frame_publisher_;
}

void system::enable_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        spdlog::critical("please call system::enable_mapping_module() after system::startup()");
    }
    // resume the mapping module
    mapper_->resume();
    // inform to the tracking module
    tracker_->set_mapping_module_status(true);
}

void system::disable_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        spdlog::critical("please call system::disable_mapping_module() after system::startup()");
    }
    // pause the mapping module
    auto future_pause = mapper_->async_pause();
    // wait until it stops
    future_pause.get();
    // inform to the tracking module
    tracker_->set_mapping_module_status(false);
}

bool system::mapping_module_is_enabled() const {
    return !mapper_->is_paused();
}

void system::enable_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    global_optimizer_->enable_loop_detector();
}

void system::disable_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    global_optimizer_->disable_loop_detector();
}

bool system::loop_detector_is_enabled() const {
    return global_optimizer_->loop_detector_is_enabled();
}

bool system::loop_BA_is_running() const {
    return global_optimizer_->loop_BA_is_running();
}

void system::abort_loop_BA() {
    global_optimizer_->abort_loop_BA();
}

std::shared_ptr<Mat44_t> system::feed_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::Monocular);

    check_reset_request();

    const auto start = std::chrono::system_clock::now();

    // color conversion
    cv::Mat img_gray = img;
    util::convert_to_grayscale(img_gray, camera_->color_order_);

    bool is_init = tracker_->tracking_state_ == tracker_state_t::NotInitialized || tracker_->tracking_state_ == tracker_state_t::Initializing;

    data::frame_observation frm_obs;

    // Extract ORB feature
    auto extractor = is_init ? ini_extractor_left_ : extractor_left_;
    extractor->extract(img_gray, mask, frm_obs.keypts_, frm_obs.descriptors_);
    frm_obs.num_keypts_ = frm_obs.keypts_.size();
    if (frm_obs.keypts_.empty()) {
        spdlog::warn("preprocess: cannot extract any keypoints");
    }

    // Undistort keypoints
    camera_->undistort_keypoints(frm_obs.keypts_, frm_obs.undist_keypts_);

    // Ignore stereo parameters
    frm_obs.stereo_x_right_ = std::vector<float>(frm_obs.num_keypts_, -1);
    frm_obs.depths_ = std::vector<float>(frm_obs.num_keypts_, -1);

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_);

    const auto cam_pose_wc = tracker_->track(data::frame(timestamp, camera_, orb_params_, frm_obs));

    const auto end = std::chrono::system_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    frame_publisher_->update(tracker_, img_gray, elapsed_ms);
    if (tracker_->tracking_state_ == tracker_state_t::Tracking && cam_pose_wc) {
        map_publisher_->set_current_cam_pose(util::converter::inverse_pose(*cam_pose_wc));
    }

    return cam_pose_wc;
}

std::shared_ptr<Mat44_t> system::feed_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::Stereo);

    check_reset_request();

    const auto start = std::chrono::system_clock::now();

    // color conversion
    cv::Mat img_gray = left_img;
    cv::Mat right_img_gray = right_img;
    util::convert_to_grayscale(img_gray, camera_->color_order_);
    util::convert_to_grayscale(right_img_gray, camera_->color_order_);

    data::frame_observation frm_obs;
    //! keypoints of stereo right image
    std::vector<cv::KeyPoint> keypts_right;
    //! ORB descriptors of stereo right image
    cv::Mat descriptors_right;

    // Extract ORB feature
    std::thread thread_left([this, &frm_obs, &img_gray, &mask]() {
        extractor_left_->extract(img_gray, mask, frm_obs.keypts_, frm_obs.descriptors_);
    });
    std::thread thread_right([this, &frm_obs, &right_img_gray, &mask, &keypts_right, &descriptors_right]() {
        extractor_right_->extract(right_img_gray, mask, keypts_right, descriptors_right);
    });
    thread_left.join();
    thread_right.join();
    frm_obs.num_keypts_ = frm_obs.keypts_.size();
    if (frm_obs.keypts_.empty()) {
        spdlog::warn("preprocess: cannot extract any keypoints");
    }

    // Undistort keypoints
    camera_->undistort_keypoints(frm_obs.keypts_, frm_obs.undist_keypts_);

    // Estimate depth with stereo match
    match::stereo stereo_matcher(extractor_left_->image_pyramid_, extractor_right_->image_pyramid_,
                                 frm_obs.keypts_, keypts_right, frm_obs.descriptors_, descriptors_right,
                                 orb_params_->scale_factors_, orb_params_->inv_scale_factors_,
                                 camera_->focal_x_baseline_, camera_->true_baseline_);
    stereo_matcher.compute(frm_obs.stereo_x_right_, frm_obs.depths_);

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_);

    const auto cam_pose_wc = tracker_->track(data::frame(timestamp, camera_, orb_params_, frm_obs));

    const auto end = std::chrono::system_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    frame_publisher_->update(tracker_, img_gray, elapsed_ms);
    if (tracker_->tracking_state_ == tracker_state_t::Tracking && cam_pose_wc) {
        map_publisher_->set_current_cam_pose(util::converter::inverse_pose(*cam_pose_wc));
    }

    return cam_pose_wc;
}

std::shared_ptr<Mat44_t> system::feed_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::RGBD);

    check_reset_request();

    const auto start = std::chrono::system_clock::now();

    // color and depth scale conversion
    cv::Mat img_gray = rgb_img;
    cv::Mat img_depth = depthmap;
    util::convert_to_grayscale(img_gray, camera_->color_order_);
    util::convert_to_true_depth(img_depth, depthmap_factor_);

    data::frame_observation frm_obs;

    // Extract ORB feature
    extractor_left_->extract(img_gray, mask, frm_obs.keypts_, frm_obs.descriptors_);
    frm_obs.num_keypts_ = frm_obs.keypts_.size();
    if (frm_obs.keypts_.empty()) {
        spdlog::warn("preprocess: cannot extract any keypoints");
    }

    // Undistort keypoints
    camera_->undistort_keypoints(frm_obs.keypts_, frm_obs.undist_keypts_);

    // Calculate disparity from depth
    // Initialize with invalid value
    frm_obs.stereo_x_right_ = std::vector<float>(frm_obs.num_keypts_, -1);
    frm_obs.depths_ = std::vector<float>(frm_obs.num_keypts_, -1);

    for (unsigned int idx = 0; idx < frm_obs.num_keypts_; idx++) {
        const auto& keypt = frm_obs.keypts_.at(idx);
        const auto& undist_keypt = frm_obs.undist_keypts_.at(idx);

        const float x = keypt.pt.x;
        const float y = keypt.pt.y;

        const float depth = img_depth.at<float>(y, x);

        if (depth <= 0) {
            continue;
        }

        frm_obs.depths_.at(idx) = depth;
        frm_obs.stereo_x_right_.at(idx) = undist_keypt.pt.x - camera_->focal_x_baseline_ / depth;
    }

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_);

    const auto cam_pose_wc = tracker_->track(data::frame(timestamp, camera_, orb_params_, frm_obs));

    const auto end = std::chrono::system_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    frame_publisher_->update(tracker_, img_gray, elapsed_ms);
    if (tracker_->tracking_state_ == tracker_state_t::Tracking && cam_pose_wc) {
        map_publisher_->set_current_cam_pose(util::converter::inverse_pose(*cam_pose_wc));
    }

    return cam_pose_wc;
}

bool system::relocalize_by_pose(const Mat44_t& cam_pose_wc) {
    const Mat44_t cam_pose_cw = util::converter::inverse_pose(cam_pose_wc);
    bool status = tracker_->request_relocalize_by_pose(cam_pose_cw);
    if (status) {
        // Even if state will be lost, still update the pose in map_publisher_
        // to clearly show new camera position
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }
    return status;
}

bool system::relocalize_by_pose_2d(const Mat44_t& cam_pose_wc, const Vec3_t& normal_vector) {
    const Mat44_t cam_pose_cw = util::converter::inverse_pose(cam_pose_wc);
    bool status = tracker_->request_relocalize_by_pose_2d(cam_pose_cw, normal_vector);
    if (status) {
        // Even if state will be lost, still update the pose in map_publisher_
        // to clearly show new camera position
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }
    return status;
}

void system::pause_tracker() {
    auto future_pause = tracker_->async_pause();
    future_pause.get();
}

bool system::tracker_is_paused() const {
    return tracker_->is_paused();
}

void system::resume_tracker() {
    tracker_->resume();
}

void system::request_reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    reset_is_requested_ = true;
}

bool system::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void system::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool system::terminate_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void system::check_reset_request() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    if (reset_is_requested_) {
        tracker_->reset();
        reset_is_requested_ = false;
    }
}

void system::pause_other_threads() const {
    // pause the mapping module
    if (mapper_ && !mapper_->is_terminated()) {
        auto future_pause = mapper_->async_pause();
        while (future_pause.wait_for(std::chrono::milliseconds(5)) == std::future_status::timeout) {
            if (mapper_->is_terminated()) {
                break;
            }
        }
    }
    // pause the global optimization module
    if (global_optimizer_ && !global_optimizer_->is_terminated()) {
        auto future_pause = global_optimizer_->async_pause();
        while (future_pause.wait_for(std::chrono::milliseconds(5)) == std::future_status::timeout) {
            if (global_optimizer_->is_terminated()) {
                break;
            }
        }
    }
}

void system::resume_other_threads() const {
    // resume the global optimization module
    if (global_optimizer_) {
        global_optimizer_->resume();
    }
    // resume the mapping module
    if (mapper_) {
        mapper_->resume();
    }
}

} // namespace openvslam
