#ifndef STELLA_VSLAM_CONFIG_H
#define STELLA_VSLAM_CONFIG_H

#include "stella_vslam/feature/orb_params.h"

#include <yaml-cpp/yaml.h>

namespace stella_vslam {

namespace marker_model {
class base;
}

class config {
public:
    //! Constructor
    explicit config(const std::string& config_file_path);
    explicit config(const YAML::Node& yaml_node, const std::string& config_file_path = "");

    //! Destructor
    ~config();

    friend std::ostream& operator<<(std::ostream& os, const config& cfg);

    //! path to config YAML file
    const std::string config_file_path_;

    //! YAML node
    const YAML::Node yaml_node_;

    //! Marker model
    std::shared_ptr<marker_model::base> marker_model_ = nullptr;
};

} // namespace stella_vslam

#endif // STELLA_VSLAM_CONFIG_H
