#include "socket_publisher/publisher.h"

#include "stella_vslam/system.h"
#include "stella_vslam/publish/frame_publisher.h"
#include "stella_vslam/util/yaml.h"

namespace socket_publisher {

publisher::publisher(const YAML::Node& yaml_node,
                     const std::shared_ptr<stella_vslam::system>& system,
                     const std::shared_ptr<stella_vslam::publish::frame_publisher>& frame_publisher,
                     const std::shared_ptr<stella_vslam::publish::map_publisher>& map_publisher)
    : system_(system),
      emitting_interval_(yaml_node["emitting_interval"].as<unsigned int>(15000)),
      image_quality_(yaml_node["image_quality"].as<unsigned int>(20)),
      client_(new socket_client(yaml_node["server_uri"].as<std::string>("http://127.0.0.1:3000"))) {
    data_serializer_ = std::unique_ptr<data_serializer>(new data_serializer(
        frame_publisher, map_publisher,
        yaml_node["publish_points"].as<bool>(true)));

    client_->set_signal_callback(std::bind(&publisher::callback, this, std::placeholders::_1));
}

void publisher::run() {
    is_terminated_ = false;
    is_paused_ = false;

    const auto serialized_reset_signal = data_serializer::serialized_reset_signal_;
    client_->emit("map_publish", serialized_reset_signal);

    while (true) {
        const auto t0 = std::chrono::system_clock::now();

        const auto serialized_map_data = data_serializer_->serialize_map_diff();
        if (!serialized_map_data.empty()) {
            client_->emit("map_publish", serialized_map_data);
        }

        const auto serialized_frame_data = data_serializer_->serialize_latest_frame(image_quality_);
        if (!serialized_frame_data.empty()) {
            client_->emit("frame_publish", serialized_frame_data);
        }

        // sleep until emitting interval time is past
        const auto t1 = std::chrono::system_clock::now();
        const auto elapse_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        if (elapse_us < emitting_interval_) {
            const auto sleep_us = emitting_interval_ - elapse_us;
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }

        if (pause_if_requested()) {
            while (is_paused()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
        }

        if (terminate_is_requested()) {
            break;
        }
    }

    system_->request_terminate();
    terminate();
}

void publisher::callback(const std::string& message) {
    if (message == "disable_mapping_mode") {
        system_->disable_mapping_module();
    }
    else if (message == "enable_mapping_mode") {
        system_->enable_mapping_module();
    }
    else if (message == "reset") {
        system_->request_reset();
    }
    else if (message == "terminate") {
        request_terminate();
    }
}

void publisher::request_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (!is_paused_) {
        pause_is_requested_ = true;
    }
}

bool publisher::is_paused() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

void publisher::resume() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    is_paused_ = false;
}

void publisher::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool publisher::is_terminated() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool publisher::terminate_is_requested() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void publisher::terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    is_terminated_ = true;
}

bool publisher::pause_if_requested() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);

    if (terminate_is_requested_) {
        return false;
    }
    else if (pause_is_requested_) {
        is_paused_ = true;
        pause_is_requested_ = false;
        return true;
    }
    return false;
}

} // namespace socket_publisher
