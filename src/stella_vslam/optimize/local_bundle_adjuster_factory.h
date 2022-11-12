#ifndef STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_FACTORY_H
#define STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_FACTORY_H

#include "stella_vslam/optimize/local_bundle_adjuster_g2o.h"
#ifdef USE_GTSAM
#include "stella_vslam/optimize/local_bundle_adjuster_gtsam.h"
#endif // USE_GTSAM

#include <memory>

namespace stella_vslam {

namespace optimize {

class local_bundle_adjuster_factory {
public:
    static std::unique_ptr<local_bundle_adjuster> create(const YAML::Node& yaml_node) {
        const auto& backend = yaml_node["backend"].as<std::string>("g2o");
        if (backend == "g2o") {
            return std::unique_ptr<local_bundle_adjuster>(new local_bundle_adjuster_g2o(yaml_node));
        }
        else if (backend == "gtsam") {
#ifdef USE_GTSAM
            return std::unique_ptr<local_bundle_adjuster>(new local_bundle_adjuster_gtsam(yaml_node));
#else
            throw std::runtime_error("gtsam is not enabled");
#endif // USE_GTSAM
        }
        else {
            throw std::runtime_error("Invalid backend");
        }
    }
};

} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_FACTORY_H
