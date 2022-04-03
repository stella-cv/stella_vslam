#include "stella_vslam/config.h"
#include "stella_vslam/camera/perspective.h"
#include "stella_vslam/camera/fisheye.h"
#include "stella_vslam/camera/equirectangular.h"
#include "stella_vslam/camera/radial_division.h"
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

    //========================//
    // Load Camera Parameters //
    //========================//

    spdlog::debug("load camera model type");
    const auto camera_model_type = camera::base::load_model_type(yaml_node_["Camera"]);

    spdlog::debug("load camera model parameters");
    try {
        switch (camera_model_type) {
            case camera::model_type_t::Perspective: {
                camera_ = new camera::perspective(yaml_node_["Camera"]);
                break;
            }
            case camera::model_type_t::Fisheye: {
                camera_ = new camera::fisheye(yaml_node_["Camera"]);
                break;
            }
            case camera::model_type_t::Equirectangular: {
                camera_ = new camera::equirectangular(yaml_node_["Camera"]);
                break;
            }
            case camera::model_type_t::RadialDivision: {
                camera_ = new camera::radial_division(yaml_node_["Camera"]);
                break;
            }
        }
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading camera model parameters: {}", e.what());
        if (camera_) {
            delete camera_;
            camera_ = nullptr;
        }
        throw;
    }

    if (camera_->setup_type_ == camera::setup_type_t::Stereo || camera_->setup_type_ == camera::setup_type_t::RGBD) {
        if (camera_->model_type_ == camera::model_type_t::Equirectangular) {
            throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
        }
    }

    spdlog::debug("load ORB parameters");
    try {
        orb_params_ = new feature::orb_params(util::yaml_optional_ref(yaml_node_, "Feature"));
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
    delete camera_;
    camera_ = nullptr;

    delete orb_params_;
    orb_params_ = nullptr;

    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    os << cfg.yaml_node_;
    return os;
}

} // namespace stella_vslam
