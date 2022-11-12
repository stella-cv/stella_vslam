#ifndef STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_FACTORY_H
#define STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_FACTORY_H

#include "stella_vslam/optimize/pose_optimizer_g2o.h"
#ifdef USE_GTSAM
#include "stella_vslam/optimize/pose_optimizer_gtsam.h"
#endif // USE_GTSAM
#include "stella_vslam/type.h"
#include "stella_vslam/util/yaml.h"

#include <memory>

namespace stella_vslam {
namespace optimize {

class pose_optimizer_factory {
public:
    static std::unique_ptr<pose_optimizer> create(const YAML::Node& yaml_node) {
        const auto& backend = yaml_node["backend"].as<std::string>("g2o");
        if (backend == "g2o") {
            return std::unique_ptr<pose_optimizer>(new pose_optimizer_g2o());
        }
        else if (backend == "gtsam") {
#ifdef USE_GTSAM
            YAML::Node gtsam_node = util::yaml_optional_ref(yaml_node, "gtsam");
            auto num_iter = gtsam_node["num_iter"].as<unsigned int>(5);
            auto relative_error_tol = gtsam_node["relative_error_tol"].as<double>(1e-2);
            auto lambda_initial = gtsam_node["lambda_initial"].as<double>(1e-5);
            auto lambda_upper_bound = gtsam_node["lambda_upper_bound"].as<double>(1e-2);
            auto enable_outlier_elimination = gtsam_node["enable_outlier_elimination"].as<bool>(true);
            auto verbosity = gtsam_node["verbosity"].as<std::string>("SILENT");
            return std::unique_ptr<pose_optimizer>(new pose_optimizer_gtsam(
                num_iter, relative_error_tol,
                lambda_initial, lambda_upper_bound,
                enable_outlier_elimination, verbosity));
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

#endif // STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_FACTORY_H
