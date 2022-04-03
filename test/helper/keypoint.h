#ifndef STELLA_VSLAM_TEST_HELPER_KEYPOINT_H
#define STELLA_VSLAM_TEST_HELPER_KEYPOINT_H

#include "stella_vslam/type.h"

using namespace stella_vslam;

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::Point2f>& keypts, const double noise_stddev = 0.0);

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::KeyPoint>& keypts, const double noise_stddev = 0.0);

#endif // STELLA_VSLAM_TEST_HELPER_KEYPOINT_H
