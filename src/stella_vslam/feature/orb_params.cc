#include "stella_vslam/feature/orb_params.h"
#include "stella_vslam/feature/default_params.h"

#include <nlohmann/json.hpp>
#include <iostream>

namespace stella_vslam {
namespace feature {

orb_params::orb_params(const std::string& name)
    : orb_params(name, "ORB", default_scale_factor, default_num_levels, default_ini_fast_thr, default_min_fast_thr) {}

orb_params::orb_params(const std::string& name, const std::string& type, const float scale_factor, const unsigned int num_levels,
                       const unsigned int ini_fast_thr, const unsigned int min_fast_thr)
    : params(name, type, scale_factor, num_levels, ini_fast_thr, min_fast_thr) {}

orb_params::orb_params(const YAML::Node& yaml_node)
    : orb_params(yaml_node["name"].as<std::string>("default ORB feature extraction setting"),
                 yaml_node["type"].as<std::string>(),
                 yaml_node["scale_factor"].as<float>(default_scale_factor),
                 yaml_node["num_levels"].as<unsigned int>(default_num_levels),
                 yaml_node["ini_fast_threshold"].as<unsigned int>(default_ini_fast_thr),
                 yaml_node["min_fast_threshold"].as<unsigned int>(default_min_fast_thr)) {}

orb_params::orb_params(const nlohmann::json& json_obj)
    : orb_params(json_obj.at("name").get<std::string>(),
                 json_obj.at("type").get<std::string>(),
                 json_obj.at("scale_factor").get<float>(),
                 json_obj.at("num_levels").get<unsigned int>(),
                 json_obj.at("ini_fast_threshold").get<unsigned int>(),
                 json_obj.at("min_fast_threshold").get<unsigned int>()) {}

nlohmann::json orb_params::to_json() const {
    return {{"name", name_},
            {"type", type_},
            {"scale_factor", scale_factor_},
            {"num_levels", num_levels_},
            {"ini_fast_threshold", ini_fast_thr_},
            {"min_fast_threshold", min_fast_thr_}};
}

std::ostream& operator<<(std::ostream& os, const orb_params& oparam) {
    os << "- scale factor: " << oparam.scale_factor_ << std::endl;
    os << "- number of levels: " << oparam.num_levels_ << std::endl;
    os << "- initial fast threshold: " << oparam.ini_fast_thr_ << std::endl;
    os << "- minimum fast threshold: " << oparam.min_fast_thr_ << std::endl;
    return os;
}

} // namespace feature
} // namespace stella_vslam
