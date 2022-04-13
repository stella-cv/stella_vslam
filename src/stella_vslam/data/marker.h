#ifndef STELLA_VSLAM_DATA_MARKER_H
#define STELLA_VSLAM_DATA_MARKER_H

#include "stella_vslam/type.h"

#include <mutex>

#include <Eigen/Core>

namespace stella_vslam {
namespace marker_model {
class base;
}

namespace data {

class keyframe;

class marker {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    marker(const eigen_alloc_vector<Vec3_t>& corners_pos_w, unsigned int id, const std::shared_ptr<marker_model::base>& marker_model);

    void set_corner_pos(const eigen_alloc_vector<Vec3_t>& corner_pos_w);

    //! corner positions
    eigen_alloc_vector<Vec3_t> corners_pos_w_;

    //! marker ID
    unsigned int id_;

    //! marker model
    std::shared_ptr<marker_model::base> marker_model_;

    //! observed keyframes
    std::vector<std::shared_ptr<keyframe>> observations_;

    mutable std::mutex mtx_position_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_MARKER_H
