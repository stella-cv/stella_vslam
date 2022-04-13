#ifndef STELLA_VSLAM_UTIL_IMAGE_CONVERTER_H
#define STELLA_VSLAM_UTIL_IMAGE_CONVERTER_H

#include "stella_vslam/camera/base.h"

#include <opencv2/core/mat.hpp>

namespace stella_vslam {
namespace util {

void convert_to_grayscale(cv::Mat& img, const camera::color_order_t in_color_order);

void convert_to_true_depth(cv::Mat& img, const double depthmap_factor);

void equalize_histogram(cv::Mat& img);

} // namespace util
} // namespace stella_vslam

#endif // STELLA_VSLAM_UTIL_IMAGE_CONVERTER_H
