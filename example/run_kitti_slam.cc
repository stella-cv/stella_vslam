#include "util/kitti_util.h"

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/util/yaml.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <numeric>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void mono_tracking(const std::shared_ptr<stella_vslam::config>& cfg,
                   const std::string& vocab_file_path,
                   const std::string& sequence_dir_path,
                   const unsigned int frame_skip,
                   const bool no_sleep,
                   const bool wait_loop_ba,
                   const bool auto_term,
                   const std::string& eval_log_dir,
                   const std::string& map_db_path,
                   const bool load_map,
                   const bool disable_mapping) {
    const kitti_sequence sequence(sequence_dir_path);
    const auto frames = sequence.get_frames();

    // build a SLAM system
    stella_vslam::system SLAM(cfg, vocab_file_path);
    bool need_initialize = true;
    if (load_map) {
        need_initialize = false;
        // load the prebuilt map
        SLAM.load_map_database(map_db_path);
    }
    SLAM.startup(need_initialize);
    if (disable_mapping) {
        SLAM.disable_mapping_module();
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    track_times.reserve(frames.size());

    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frames.size(); ++i) {
            // wait until the loop BA is finished
            if (wait_loop_ba) {
                while (SLAM.loop_BA_is_running() || !SLAM.mapping_module_is_enabled()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            }

            const auto& frame = frames.at(i);
            const auto img = cv::imread(frame.left_img_path_, cv::IMREAD_UNCHANGED);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                SLAM.feed_monocular_frame(img, frame.timestamp_);
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < frames.size() - 1) {
                const auto wait_time = frames.at(i + 1).timestamp_ - (frame.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

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

    if (!eval_log_dir.empty()) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory(eval_log_dir + "/frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory(eval_log_dir + "/keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs(eval_log_dir + "/track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

void stereo_tracking(const std::shared_ptr<stella_vslam::config>& cfg,
                     const std::string& vocab_file_path,
                     const std::string& sequence_dir_path,
                     const unsigned int frame_skip,
                     const bool no_sleep,
                     const bool wait_loop_ba,
                     const bool auto_term,
                     const std::string& eval_log_dir,
                     const std::string& map_db_path,
                     const bool load_map,
                     const bool disable_mapping) {
    const kitti_sequence sequence(sequence_dir_path);
    const auto frames = sequence.get_frames();

    // build a SLAM system
    stella_vslam::system SLAM(cfg, vocab_file_path);
    bool need_initialize = true;
    if (load_map) {
        need_initialize = false;
        // load the prebuilt map
        SLAM.load_map_database(map_db_path);
    }
    SLAM.startup(need_initialize);
    if (disable_mapping) {
        SLAM.disable_mapping_module();
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    track_times.reserve(frames.size());

    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frames.size(); ++i) {
            // wait until the loop BA is finished
            if (wait_loop_ba) {
                while (SLAM.loop_BA_is_running() || !SLAM.mapping_module_is_enabled()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            const auto& frame = frames.at(i);
            const auto left_img = cv::imread(frame.left_img_path_, cv::IMREAD_UNCHANGED);
            const auto right_img = cv::imread(frame.right_img_path_, cv::IMREAD_UNCHANGED);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!left_img.empty() && !right_img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                SLAM.feed_stereo_frame(left_img, right_img, frame.timestamp_);
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < frames.size() - 1) {
                const auto wait_time = frames.at(i + 1).timestamp_ - (frame.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

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

    if (!eval_log_dir.empty()) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory(eval_log_dir + "/frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory(eval_log_dir + "/keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs(eval_log_dir + "/track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

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
    auto data_dir_path = op.add<popl::Value<std::string>>("d", "data-dir", "directory path which contains dataset");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto wait_loop_ba = op.add<popl::Switch>("", "wait-loop-ba", "wait until the loop BA is finished");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto log_level = op.add<popl::Value<std::string>>("", "log-level", "log level", "info");
    auto eval_log_dir = op.add<popl::Value<std::string>>("", "eval-log-dir", "store trajectory and tracking times at this path (Specify the directory where it exists.)", "");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    auto load_map = op.add<popl::Switch>("", "load-map", "load a map database");
    auto disable_mapping = op.add<popl::Switch>("", "disable-mapping", "disable mapping");
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
    if (!vocab_file_path->is_set() || !data_dir_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level->value()));

    // load configuration
    std::shared_ptr<stella_vslam::config> cfg;
    try {
        cfg = std::make_shared<stella_vslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg,
                      vocab_file_path->value(),
                      data_dir_path->value(),
                      frame_skip->value(),
                      no_sleep->is_set(),
                      wait_loop_ba->is_set(),
                      auto_term->is_set(),
                      eval_log_dir->value(),
                      map_db_path->value(),
                      load_map->is_set(),
                      disable_mapping->is_set());
    }
    else if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        stereo_tracking(cfg,
                        vocab_file_path->value(),
                        data_dir_path->value(),
                        frame_skip->value(),
                        no_sleep->is_set(),
                        wait_loop_ba->is_set(),
                        auto_term->is_set(),
                        eval_log_dir->value(),
                        map_db_path->value(),
                        load_map->is_set(),
                        disable_mapping->is_set());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
