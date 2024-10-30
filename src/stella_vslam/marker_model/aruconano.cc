#include "stella_vslam/marker_model/aruconano.h"

#include <iostream>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace marker_model {

aruconano::aruconano(double width, int dict)
    : base(width), dict_(dict) {
    spdlog::debug("CONSTRUCT: marker_model::aruconano");
}

aruconano::~aruconano() {
    spdlog::debug("DESTRUCT: marker_model::aruconano");
}

nlohmann::json aruconano::to_json() const {
    return {
        {"width", width_},
        {"dict", dict_},
    };
}

std::ostream& operator<<(std::ostream& os, const aruconano& params) {
    os << "- width: " << params.width_ << std::endl;
    os << "- dict: " << params.dict_ << std::endl;
    os << "- corners_pos: " << std::endl;
    for (auto& corner_pos : params.corners_pos_) {
        os << "- " << corner_pos << std::endl;
    }
    return os;
}

} // namespace marker_model
} // namespace stella_vslam
