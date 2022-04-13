#ifndef STELLA_VSLAM_SOLVE_UTIL_H
#define STELLA_VSLAM_SOLVE_UTIL_H

#include "stella_vslam/type.h"

#include <vector>

#include <opencv2/core/types.hpp>

namespace stella_vslam {
namespace solve {

void normalize(const std::vector<cv::KeyPoint>& keypts, std::vector<cv::Point2f>& normalized_pts, Mat33_t& transform);

} // namespace solve
} // namespace stella_vslam

#endif // STELLA_VSLAM_SOLVE_UTIL_H
