#ifndef STELLA_VSLAM_CAMERA_CAMERA_FACTORY_H
#define STELLA_VSLAM_CAMERA_CAMERA_FACTORY_H

#include "stella_vslam/camera/base.h"
#include "stella_vslam/camera/perspective.h"
#include "stella_vslam/camera/fisheye.h"
#include "stella_vslam/camera/equirectangular.h"
#include "stella_vslam/camera/radial_division.h"

#include <spdlog/spdlog.h>

namespace stella_vslam {

namespace camera {

class camera_factory {
public:
    static camera::base* create(const YAML::Node& node) {
        const auto camera_model_type = camera::base::load_model_type(node);

        camera::base* camera = nullptr;
        try {
            switch (camera_model_type) {
                case camera::model_type_t::Perspective: {
                    camera = new camera::perspective(node);
                    break;
                }
                case camera::model_type_t::Fisheye: {
                    camera = new camera::fisheye(node);
                    break;
                }
                case camera::model_type_t::Equirectangular: {
                    camera = new camera::equirectangular(node);
                    break;
                }
                case camera::model_type_t::RadialDivision: {
                    camera = new camera::radial_division(node);
                    break;
                }
            }
        }
        catch (const std::exception& e) {
            spdlog::debug("failed in loading camera model parameters: {}", e.what());
            if (camera) {
                delete camera;
                camera = nullptr;
            }
            throw;
        }

        assert(camera != nullptr);
        if (camera->setup_type_ == camera::setup_type_t::Stereo || camera->setup_type_ == camera::setup_type_t::RGBD) {
            if (camera->model_type_ == camera::model_type_t::Equirectangular) {
                throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
            }
        }
        return camera;
    }
};

} // namespace camera
} // namespace stella_vslam

#endif // STELLA_VSLAM_CAMERA_CAMERA_FACTORY_H
