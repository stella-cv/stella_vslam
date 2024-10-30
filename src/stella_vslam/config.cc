#include "stella_vslam/config.h"
#include "stella_vslam/marker_model/aruco.h"
#ifdef USE_ARUCO_NANO
#include "stella_vslam/marker_model/aruconano.h"
#endif // USE_ARUCO_NANO
#include "stella_vslam/util/string.h"
#include "stella_vslam/util/yaml.h"

#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

namespace stella_vslam {

config::config(const std::string& config_file_path)
    : config(YAML::LoadFile(config_file_path), config_file_path) {}

config::config(const YAML::Node& yaml_node, const std::string& config_file_path)
    : config_file_path_(config_file_path), yaml_node_(yaml_node) {
    spdlog::debug("CONSTRUCT: config");

    spdlog::info("config file loaded: {}", config_file_path_);

    //========================//
    // Load Marker Parameters //
    //========================//

    auto marker_model_yaml_node = yaml_node_["MarkerModel"];
    if (marker_model_yaml_node) {
        spdlog::debug("load marker model parameters");
        auto marker_model_type = marker_model_yaml_node["type"].as<std::string>();
        if (marker_model_type == "aruco") {
            marker_model_ = std::make_shared<marker_model::aruco>(
                marker_model_yaml_node["width"].as<double>(),
                marker_model_yaml_node["marker_size"].as<double>(),
                marker_model_yaml_node["max_markers"].as<double>());
        }

#ifdef USE_ARUCO_NANO
        else if (marker_model_type == "aruconano") {
            marker_model_ = std::make_shared<marker_model::aruconano>(
                marker_model_yaml_node["width"].as<double>(),
                marker_model_yaml_node["dict"].as<int>(0));
        }
#endif // USE_ARUCO_NANO

        else {
            throw std::runtime_error("Invalid marker model type :" + marker_model_type);
        }
    }
}

config::~config() {
    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    os << cfg.yaml_node_;
    return os;
}

} // namespace stella_vslam
