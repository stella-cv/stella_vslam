#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/util/yaml.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>

#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

int run(const std::shared_ptr<stella_vslam::config>& cfg,
        const int keyfrm1_id,
        const int keyfrm2_id,
        const std::string& vocab_file_path,
        const bool auto_term,
        const bool eval_log,
        const std::string& map_db_path,
        const bool disable_gui) {
    // build a SLAM system
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_file_path);
    bool need_initialize = false;
    // load the prebuilt map
    if (!slam->load_map_database(map_db_path)) {
        return EXIT_FAILURE;
    }
    slam->startup(need_initialize);
    slam->disable_mapping_module();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), slam, slam->get_frame_publisher(), slam->get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), slam, slam->get_frame_publisher(), slam->get_map_publisher());
#endif

    slam->request_loop_closure(keyfrm1_id, keyfrm2_id);

    // run the SLAM in another thread
    std::thread thread([&]() {
        // wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        if (!disable_gui) {
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
        }
    });

    if (!disable_gui) {
        // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
        viewer.run();
#elif USE_SOCKET_PUBLISHER
        publisher.run();
#endif
    }

    thread.join();

    // shutdown the SLAM process
    slam->shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        slam->save_frame_trajectory("frame_trajectory.txt", "TUM");
        slam->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
    }

    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    backward::SignalHandling sh;
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto keyfrm_id1 = op.add<popl::Value<int>>("a", "keyfrm-id1", "keyframe ID 1");
    auto keyfrm_id2 = op.add<popl::Value<int>>("b", "keyfrm-id2", "keyframe ID 2");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto log_level = op.add<popl::Value<std::string>>("", "log-level", "log level", "info");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    auto disable_gui = op.add<popl::Switch>("", "disable-gui", "run without GUI");
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
    if (!op.unknown_options().empty()) {
        for (const auto& unknown_option : op.unknown_options()) {
            std::cerr << "unknown_options: " << unknown_option << std::endl;
        }
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !config_file_path->is_set()) {
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
    int ret = run(cfg,
                  keyfrm_id1->value(),
                  keyfrm_id2->value(),
                  vocab_file_path->value(),
                  auto_term->is_set(),
                  eval_log->is_set(),
                  map_db_path->value(),
                  disable_gui->value());

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return ret;
}
