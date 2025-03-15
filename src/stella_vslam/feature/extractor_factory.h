#ifndef STELLA_VSLAM_FEATURE_FEATURE_FACTORY_H
#define STELLA_VSLAM_FEATURE_FEATURE_FACTORY_H
#include "stella_vslam/feature/extractor.h"
#include "stella_vslam/feature/orb_extractor.h"
#include "stella_vslam/feature/sift_extractor.h"
#include "stella_vslam/feature/surf_extractor.h"
#include "stella_vslam/feature/params.h"
#include "stella_vslam/util/yaml.h"

#include "yaml-cpp/yaml.h"

namespace stella_vslam {
namespace feature {
class extractor_factory {
public:
    static extractor* create(const YAML::Node& node, const feature_params* params) {
        const auto feature_type = extractor::load_feature_type(node["Feature"]);
        extractor* extractor = nullptr;
        try {
            switch (feature_type) {
                case feature_types::ORB: {
                    const auto preprocessing_params = node["Preprocessing"];
                    auto mask_rectangles = util::get_rectangles(preprocessing_params["mask_rectangles"]);

                    const auto min_size = preprocessing_params["min_size"].as<unsigned int>(800);
                    extractor = new feature::orb_extractor(std::make_shared<feature::feature_params>(*params), min_size, mask_rectangles);
                    break;
                }
                case feature_types::sift: {
                    extractor = new sift_extractor(std::make_shared<feature::feature_params>(*params));
                    break;
                }
                case feature_types::surf: {
                    extractor = new surf_extractor(std::make_shared<feature::feature_params>(*params));
                    break;
                }
            }
        }
        catch (const std::exception& e) {
            spdlog::debug("failed in loading extractor parameters: {}", e.what());
            if (extractor) {
                delete extractor;
                extractor = nullptr;
            }
            throw;
        }

        assert(extractor != nullptr);

        return extractor;
    }
};
} // namespace feature
} // namespace stella_vslam
#endif // STELLA_VSLAM_FEATURE_FEATURE_FACTORY_H