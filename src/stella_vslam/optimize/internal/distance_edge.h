#ifndef STELLA_VSLAM_OPTIMIZER_G2O_DISTANCE_EDGE_H
#define STELLA_VSLAM_OPTIMIZER_G2O_DISTANCE_EDGE_H

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/internal/landmark_vertex.h"

#include <g2o/core/base_binary_edge.h>

namespace stella_vslam {
namespace optimize {
namespace internal {

class distance_edge final : public g2o::BaseBinaryEdge<1, double, landmark_vertex, landmark_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    distance_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;
};

inline distance_edge::distance_edge()
    : g2o::BaseBinaryEdge<1, double, landmark_vertex, landmark_vertex>() {}

inline bool distance_edge::read(std::istream& is) {
    is >> _measurement;
    is >> information()(0, 0);
    return true;
}

inline bool distance_edge::write(std::ostream& os) const {
    os << measurement() << " ";
    os << " " << information()(0, 0);
    return os.good();
}

inline void distance_edge::computeError() {
    const auto v1 = static_cast<const landmark_vertex*>(_vertices.at(0));
    const auto v2 = static_cast<const landmark_vertex*>(_vertices.at(1));
    _error[0] = _measurement - (v2->estimate() - v1->estimate()).norm();
}

} // namespace internal
} // namespace optimize
} // namespace stella_vslam

#endif // STELLA_VSLAM_OPTIMIZER_G2O_DISTANCE_EDGE_H
