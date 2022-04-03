#include "stella_vslam/data/marker.h"

namespace stella_vslam {
namespace data {

marker::marker(const eigen_alloc_vector<Vec3_t>& corners_pos_w, unsigned int id, const std::shared_ptr<marker_model::base>& marker_model)
    : corners_pos_w_(corners_pos_w), id_(id), marker_model_(marker_model) {}

void marker::set_corner_pos(const eigen_alloc_vector<Vec3_t>& corner_pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    corners_pos_w_ = corner_pos_w;
}

} // namespace data
} // namespace stella_vslam
