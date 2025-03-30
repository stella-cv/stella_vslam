#ifndef STELLA_VSLAM_FEATURE_PARAMS_FACTORY_H
#define STELLA_VSLAM_FEATURE_PARAMS_FACTORY_H
#include "stella_vslam/feature/params.h"
#include "stella_vslam/feature/orb_params.h"
#include "stella_vslam/feature/sift_params.h"
#include "stella_vslam/feature/akaze_params.h"
#include "stella_vslam/feature/feature_type.h"
#include "stella_vslam/util/yaml.h"

#include <nlohmann/json.hpp>
#include "yaml-cpp/yaml.h"

namespace stella_vslam {
namespace feature {
class params_factory {
public:
    static params* create(const YAML::Node& node) {
        const auto feature_type = load_feature_type(node);
        params* params = nullptr;
        try {
            switch (feature_type) {
                case feature_types::ORB: {
                    params = new orb_params(node);
                    break;
                }
                case feature_types::SIFT: {
                    params = new sift_params(node);
                    break;
                }
                case feature_types::AKAZE: {
                    params = new akaze_params(node);
                    break;
                }
            }
        }
        catch (const std::exception& e) {
            spdlog::debug("failed in loading extractor parameters: {}", e.what());
            if (params) {
                delete params;
                params = nullptr;
            }
            throw;
        }

        assert(params != nullptr);

        return params;
    }

    static params* create(const nlohmann::json& json_obj) {
        const auto feature_type = load_feature_type(json_obj);
        params* params = nullptr;
        try {
            switch (feature_type) {
                case feature_types::ORB: {
                    params = new orb_params(json_obj);
                    break;
                }
                case feature_types::SIFT: {
                    params = new sift_params(json_obj);
                    break;
                }
                case feature_types::AKAZE: {
                    params = new akaze_params(json_obj);
                    break;
                }
            }
        }
        catch (const std::exception& e) {
            spdlog::debug("failed in loading extractor parameters: {}", e.what());
            if (params) {
                delete params;
                params = nullptr;
            }
            throw;
        }

        assert(params != nullptr);

        return params;
    }
};
} // namespace feature
} // namespace stella_vslam
#endif // STELLA_VSLAM_FEATURE_PARAMS_FACTORY_H