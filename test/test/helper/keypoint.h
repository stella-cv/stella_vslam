#ifndef STELLA_VSLAM_TEST_HELPER_KEYPOINT_H
#define STELLA_VSLAM_TEST_HELPER_KEYPOINT_H

#include "stella_vslam/type.h"

using namespace stella_vslam;

void create_random_keypoints(unsigned int cols, unsigned int rows, unsigned int num_points, std::vector<cv::Point2f>& keypts);

void create_random_keypoints(unsigned int cols, unsigned int rows, unsigned int num_points, std::vector<cv::KeyPoint>& keypts);

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::Point2f>& keypts);

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::KeyPoint>& keypts);

void add_noise(std::vector<cv::Point2f>& keypts, const double noise_stddev, const double ratio);

void add_noise(std::vector<cv::KeyPoint>& keypts, const double noise_stddev, const double ratio);

#endif // STELLA_VSLAM_TEST_HELPER_KEYPOINT_H
