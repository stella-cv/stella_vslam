#include "stella_vslam/config.h"
#include "stella_vslam/marker_model/aruco.h"
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

    spdlog::debug("load ORB parameters");
    try {
        orb_params_ = new feature::orb_params(util::yaml_optional_ref(yaml_node_, "Feature"));
        spdlog::info("load orb_params \"{}\"", orb_params_->name_);
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading ORB feature extraction model: {}", e.what());
        if (orb_params_) {
            delete orb_params_;
            orb_params_ = nullptr;
        }
        throw;
    }

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
        else {
            throw std::runtime_error("Invalid marker model type :" + marker_model_type);
        }
    }
}

config::~config() {
    delete orb_params_;
    orb_params_ = nullptr;

    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    os << cfg.yaml_node_;
    return os;
}

} // namespace stella_vslam
