#ifndef OPENVSLAM_UTIL_YAML_H
#define OPENVSLAM_UTIL_YAML_H

#include <string>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

namespace openvslam {
namespace util {

inline YAML::Node yaml_optional_ref(const YAML::Node& ref_node, const std::string& key) {
    return ref_node[key] ? ref_node[key] : YAML::Node();
}

} // namespace util
} // namespace openvslam

#endif // OPENVSLAM_UTIL_YAML_H
