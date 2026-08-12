#ifndef STELLA_VSLAM_MATCH_BASE_H
#define STELLA_VSLAM_MATCH_BASE_H

#include "stella_vslam/type.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <numeric>

#include <opencv2/core/mat.hpp>

namespace stella_vslam {
namespace match {

static constexpr unsigned int HAMMING_DIST_THR_LOW = 50;
static constexpr unsigned int HAMMING_DIST_THR_HIGH = 100;
static constexpr unsigned int MAX_HAMMING_DIST = 256;

/// Compute L2 distance between two float descriptors, scaled to hamming range.
/// Used for LiftFeat descriptors (L2-normalized, CV_32F).
/// @param desc_1 First descriptor (CV_32F)
/// @param desc_2 Second descriptor (CV_32F)
/// @return Scaled distance in range [0, 256]
inline unsigned int compute_descriptor_distance_float(const cv::Mat& desc_1, const cv::Mat& desc_2) {
    const float* p_a = desc_1.ptr<float>();
    const float* p_b = desc_2.ptr<float>();
    const int dims = desc_1.cols; // dimensionality is in cols

    float dist_sq = 0.0f;
    for (int i = 0; i < dims; ++i) {
        float d = p_a[i] - p_b[i];
        dist_sq += d * d;
    }
    float l2_dist = std::sqrt(dist_sq); // 0.0 ~ 2.0 (L2-normalized)

    // Convert to hamming scale (0~256)
    unsigned int scaled_dist = static_cast<unsigned int>(l2_dist * 128.0f);
    return std::min(scaled_dist, MAX_HAMMING_DIST);
}

/// Compute Hamming distance between two 256-bit binary descriptors using uint32_t elements.
/// Used for ORB / HashSIFT descriptors (8 x uint32_t = 256 bit).
/// @ref http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
/// @param desc_1 First descriptor (CV_8U, 32 cols)
/// @param desc_2 Second descriptor (CV_8U, 32 cols)
/// @return Hamming distance in range [0, 256]
inline unsigned int compute_descriptor_distance_binary_32(const cv::Mat& desc_1, const cv::Mat& desc_2) {
    constexpr uint32_t mask_1 = 0x55555555U;
    constexpr uint32_t mask_2 = 0x33333333U;
    constexpr uint32_t mask_3 = 0x0F0F0F0FU;
    constexpr uint32_t mask_4 = 0x01010101U;

    const auto* pa = desc_1.ptr<uint32_t>();
    const auto* pb = desc_2.ptr<uint32_t>();

    unsigned int dist = 0;

    for (unsigned int i = 0; i < 8; ++i, ++pa, ++pb) {
        auto v = *pa ^ *pb;
        v -= ((v >> 1) & mask_1);
        v = (v & mask_2) + ((v >> 2) & mask_2);
        dist += (((v + (v >> 4)) & mask_3) * mask_4) >> 24;
    }

    return dist;
}

/// Compute Hamming distance between two 256-bit binary descriptors using uint64_t elements.
/// (4 x uint64_t = 256 bit)
/// @ref https://stackoverflow.com/questions/21826292/t-sql-hamming-distance-function-capable-of-decimal-string-uint64?lq=1
/// @param desc_1 First descriptor (CV_8U, 64 cols)
/// @param desc_2 Second descriptor (CV_8U, 64 cols)
/// @return Hamming distance in range [0, 256]
inline unsigned int compute_descriptor_distance_binary_64(const cv::Mat& desc_1, const cv::Mat& desc_2) {
    constexpr uint64_t mask_1 = 0x5555555555555555UL;
    constexpr uint64_t mask_2 = 0x3333333333333333UL;
    constexpr uint64_t mask_3 = 0x0F0F0F0F0F0F0F0FUL;
    constexpr uint64_t mask_4 = 0x0101010101010101UL;

    const auto* pa = desc_1.ptr<uint64_t>();
    const auto* pb = desc_2.ptr<uint64_t>();

    unsigned int dist = 0;

    for (unsigned int i = 0; i < 4; ++i, ++pa, ++pb) {
        auto v = *pa ^ *pb;
        v -= (v >> 1) & mask_1;
        v = (v & mask_2) + ((v >> 2) & mask_2);
        dist += (((v + (v >> 4)) & mask_3) * mask_4) >> 56;
    }

    return dist;
}

/// Dispatch distance computation based on descriptor type.
/// - CV_32F (LiftFeat): L2 distance scaled to hamming range
/// - CV_8U (8xuint32): Hamming distance (ORB / HashSIFT)
/// - CV_8U (4xuint64): Hamming distance (64-bit representation)
/// @param desc_1 First descriptor
/// @param desc_2 Second descriptor
/// @return Distance in range [0, 256]
inline unsigned int compute_descriptor_distance(const cv::Mat& desc_1, const cv::Mat& desc_2) {
    if (desc_1.type() == CV_32F) {
        return compute_descriptor_distance_float(desc_1, desc_2);
    }
    else if (desc_1.cols == 8) {
        return compute_descriptor_distance_binary_32(desc_1, desc_2);
    }
    else {
        return compute_descriptor_distance_binary_64(desc_1, desc_2);
    }
}

inline bool check_epipolar_constraint(const Vec3_t& bearing_1, const Vec3_t& bearing_2,
                                      const Mat33_t& E_12, float residual_rad_thr,
                                      const float bearing_1_scale_factor) {
    // Normal vector of the epipolar plane on keyframe 1
    const Vec3_t epiplane_in_1 = E_12 * bearing_2;

    // Acquire the angle formed by the normal vector and the bearing
    const auto cos_residual = std::min(1.0, std::max(-1.0, epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm()));
    const auto residual_rad = std::abs(M_PI / 2.0 - std::acos(cos_residual));

    // The larger keypoint scale permits less constraints
    return residual_rad < residual_rad_thr * bearing_1_scale_factor;
}

class base {
public:
    base(const float lowe_ratio, const bool check_orientation)
        : lowe_ratio_(lowe_ratio), check_orientation_(check_orientation) {}

    virtual ~base() = default;

protected:
    const float lowe_ratio_;
    const bool check_orientation_;
};

} // namespace match
} // namespace stella_vslam

#endif // STELLA_VSLAM_MATCH_BASE_H
