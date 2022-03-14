#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/yaml.h"

#include <cmath>
#include <iostream>
#include <chrono>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void mono_localization(const std::shared_ptr<openvslam::config>& cfg,
                       const std::string& vocab_file_path, const std::string& video_file_path, const std::string& mask_img_path,
                       const std::string& map_db_path, const std::string& map_db_path2, const bool mapping,
                       const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
                       const double scale, const std::vector<double> rotation_xyz, const std::vector<double> translation_xyz) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // load the prebuilt map
    SLAM.load_map_database(map_db_path);
    // if another map is passed load an merge it
    if (!map_db_path2.empty()){
        // Define Map Scale Factor
        double map_scale = scale;
        // Define Map Rotation with WC coordinates from Euler angles
        openvslam::Mat33_t rotation_matrix;
        rotation_matrix =   Eigen::AngleAxisd(rotation_xyz[0], Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(rotation_xyz[1], Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(rotation_xyz[2], Eigen::Vector3d::UnitZ());

        // Define Map Translation with WC coordinates
        openvslam::Vec3_t translation {translation_xyz[0], translation_xyz[1], translation_xyz[2]};
        
        // Call load_new_map_to_merge
        openvslam::Mat44_t transf_matrix = openvslam::Mat44_t::Identity();
        transf_matrix.block<3, 3>(0, 0) = rotation_matrix;
        transf_matrix.block<3, 1>(0, 3) = translation;
        SLAM.load_new_map_database(map_db_path2, transf_matrix, map_scale);
    }
    // startup the SLAM process (it does not need initialization of a map)
    SLAM.startup(false);
    // select to activate the mapping module or not
    if (mapping) {
        SLAM.enable_mapping_module();
    }
    else {
        SLAM.disable_mapping_module();
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        openvslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    auto video = cv::VideoCapture(video_file_path, cv::CAP_FFMPEG);
    std::vector<double> track_times;

    cv::Mat frame;
    double timestamp = 0.0;

    unsigned int num_frame = 0;

    bool is_not_end = true;
    // run the SLAM in another thread
    std::thread thread([&]() {
        while (is_not_end) {
            is_not_end = video.read(frame);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!frame.empty() && (num_frame % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                SLAM.feed_monocular_frame(frame, timestamp, mask);
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (num_frame % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep) {
                const auto wait_time = 1.0 / cfg->camera_->fps_ - track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;

            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    backward::SignalHandling sh;
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto video_file_path = op.add<popl::Value<std::string>>("m", "video", "video file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "path to a prebuilt map database");
    auto map_db_path2 = op.add<popl::Value<std::string>>("", "map-db2", "path to another prebuilt map database");
    auto map_2_scale = op.add<popl::Value<double>>("s", "map-scale", "path to another prebuilt map database");
    auto map_2_rotation_x = op.add<popl::Value<double>>("", "map-rotation-x", "Euler angle of X coordinate value to rotate map-db2 in rad");
    auto map_2_rotation_y = op.add<popl::Value<double>>("", "map-rotation-y", "Euler angle of Y coordinate value to rotate map-db2 in rad");
    auto map_2_rotation_z = op.add<popl::Value<double>>("", "map-rotation-z", "Euler angle of Z coordinate value to rotate map-db2 in rad");
    auto map_2_translation_x = op.add<popl::Value<double>>("", "map-translation-x", "X coordinate value to translate map-db2");
    auto map_2_translation_y = op.add<popl::Value<double>>("", "map-translation-y", "Y coordinate value to translate map-db2");
    auto map_2_translation_z = op.add<popl::Value<double>>("", "map-translation-z", "Z coordinate value to translate map-db2");
    auto mapping = op.add<popl::Switch>("", "mapping", "perform mapping as well as localization");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !video_file_path->is_set()
        || !config_file_path->is_set() || !map_db_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // transformation of second map
    if (!map_2_scale->is_set()) {
        map_2_scale->set_value(1.0);
    }
    if (!map_2_rotation_x->is_set()) {
        map_2_rotation_x->set_value(0);
    }
    if (!map_2_rotation_y->is_set()) {
        map_2_rotation_y->set_value(0);
    }
    if (!map_2_rotation_z->is_set()) {
        map_2_rotation_z->set_value(0);
    }
    if (!map_2_translation_x->is_set()) {
        map_2_translation_x->set_value(0);
    }
    if (!map_2_translation_y->is_set()) {
        map_2_translation_y->set_value(0);
    }
    if (!map_2_translation_z->is_set()) {
        map_2_translation_z->set_value(0);
    }
    
    std::vector<double> rotation_xyz = {map_2_rotation_x->value(), map_2_rotation_y->value(), map_2_rotation_z->value()};
    std::vector<double> translation_xyz = {map_2_translation_x->value(), map_2_translation_y->value(), map_2_translation_z->value()};


#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run localization
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_localization(cfg, vocab_file_path->value(), video_file_path->value(), mask_img_path->value(),
                          map_db_path->value(), map_db_path2->value(), mapping->is_set(),
                          frame_skip->value(), no_sleep->is_set(), auto_term->is_set(), 
                          map_2_scale->value(), rotation_xyz, translation_xyz);
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
