#ifndef STELLA_VSLAM_FEATURE_FEATURE_TYPES_H
#define STELLA_VSLAM_FEATURE_FEATURE_TYPES_H
#include <nlohmann/json.hpp>
#include "yaml-cpp/yaml.h"

namespace stella_vslam {
namespace feature {
enum class feature_types {
    ORB,
    SIFT,
    AKAZE
};

//! Load model type from string
static feature_types load_feature_type(const std::string& type) {
    auto feature_type_str = type;
    if (feature_type_str == "ORB") {
        return feature_types::ORB;
    }
    else if (feature_type_str == "SIFT") {
        return feature_types::SIFT;
    }
    else if (feature_type_str == "AKAZE") {
        return feature_types::AKAZE;
    }
    throw std::runtime_error("Invalid feature type: " + feature_type_str + ". Support: ORB, SIFT and AKAZE");
}

//! Load model type from YAML
static feature_types load_feature_type(const YAML::Node& yaml_node) {
    auto feature_type_str = yaml_node["type"].as<std::string>();
    return load_feature_type(feature_type_str);
}

//! Load model type from json
static feature_types load_feature_type(const nlohmann::json& json_obj) {
    auto feature_type_str = json_obj.at("type").get<std::string>();
    return load_feature_type(feature_type_str);
}
} // namespace feature
} // namespace stella_vslam
#endif // STELLA_VSLAM_FEATURE_FEATURE_TYPES_H