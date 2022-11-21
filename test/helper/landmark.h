#ifndef STELLA_VSLAM_TEST_HELPER_LANDMARK_H
#define STELLA_VSLAM_TEST_HELPER_LANDMARK_H

#include "stella_vslam/type.h"

#include <random>

using namespace stella_vslam;

eigen_alloc_vector<Vec3_t> create_landmarks_on_plane(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix,
                                                     const std::vector<cv::Point2f>& keypts, const Vec4_t& plane_coeffs);

eigen_alloc_vector<Vec3_t> create_landmarks_on_plane(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix,
                                                     const std::vector<cv::KeyPoint>& keypts, const Vec4_t& plane_coeffs);

eigen_alloc_vector<Vec3_t> create_random_landmarks_in_space(const unsigned int num_landmarks,
                                                            const float space_lim);

eigen_alloc_vector<Vec3_t> create_random_landmarks_on_plane(const unsigned int num_landmarks,
                                                            const float space_lim, const Vec4_t& plane_coeffs);

#endif // STELLA_VSLAM_TEST_HELPER_LANDMARK_H
