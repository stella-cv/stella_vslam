#ifndef STELLA_VSLAM_UTIL_YAML_H
#define STELLA_VSLAM_UTIL_YAML_H

#include <string>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace util {

inline YAML::Node yaml_optional_ref(const YAML::Node& ref_node, const std::string& key) {
    return ref_node[key] ? ref_node[key] : YAML::Node();
}

std::vector<std::vector<float>> get_rectangles(const YAML::Node& node);

} // namespace util
} // namespace stella_vslam

#endif // STELLA_VSLAM_UTIL_YAML_H
