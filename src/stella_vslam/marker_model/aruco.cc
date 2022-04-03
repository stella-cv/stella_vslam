#include "stella_vslam/marker_model/aruco.h"

#include <iostream>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace marker_model {

aruco::aruco(double width, int marker_size, int max_markers)
    : base(width), marker_size_(marker_size), max_markers_(max_markers) {
    spdlog::debug("CONSTRUCT: marker_model::aruco");
}

aruco::~aruco() {
    spdlog::debug("DESTRUCT: marker_model::aruco");
}

nlohmann::json aruco::to_json() const {
    return {
        {"width", width_},
        {"marker_size", marker_size_},
        {"max_markers", max_markers_},
    };
}

std::ostream& operator<<(std::ostream& os, const aruco& params) {
    os << "- width: " << params.width_ << std::endl;
    os << "- marker_size: " << params.marker_size_ << std::endl;
    os << "- max_markers: " << params.max_markers_ << std::endl;
    os << "- corners_pos: " << std::endl;
    for (auto& corner_pos : params.corners_pos_) {
        os << "- " << corner_pos << std::endl;
    }
    return os;
}

} // namespace marker_model
} // namespace stella_vslam
