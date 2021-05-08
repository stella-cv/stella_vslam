#include "openvslam/config.h"
#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"
#include "openvslam/camera/radial_division.h"
#include "openvslam/util/string.h"

#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

namespace openvslam {

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
        delete camera_;
        camera_ = nullptr;
        throw;
    }

    if (camera_->setup_type_ == camera::setup_type_t::Stereo || camera_->setup_type_ == camera::setup_type_t::RGBD) {
        if (camera_->model_type_ == camera::model_type_t::Equirectangular) {
            throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
        }
    }
}

config::~config() {
    delete camera_;
    camera_ = nullptr;

    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    os << cfg.yaml_node_;
    return os;
}

} // namespace openvslam
