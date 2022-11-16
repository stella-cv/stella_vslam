#include "helper/keypoint.h"

#include <random>

void create_random_keypoints(unsigned int cols, unsigned int rows, unsigned int num_points, std::vector<cv::Point2f>& keypts) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::uniform_real_distribution<float> rand_x(0, cols);
    std::uniform_real_distribution<float> rand_y(0, rows);

    keypts.resize(num_points);
    for (unsigned int i = 0; i < num_points; ++i) {
        keypts.at(i).x = rand_x(mt);
        keypts.at(i).y = rand_y(mt);
    }
}

void create_random_keypoints(unsigned int cols, unsigned int rows, unsigned int num_points, std::vector<cv::KeyPoint>& keypts) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::uniform_real_distribution<float> rand_x(0, cols);
    std::uniform_real_distribution<float> rand_y(0, rows);

    keypts.resize(num_points);
    for (unsigned int i = 0; i < num_points; ++i) {
        keypts.at(i).pt.x = rand_x(mt);
        keypts.at(i).pt.y = rand_y(mt);
    }
}

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::Point2f>& keypts) {
    const auto num_landmarks = landmarks.size();

    // convert a 3D point to a bearing vector

    keypts.resize(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t& pos_w = landmarks.at(i);
        Vec3_t pos_c = rot_cw * pos_w + trans_cw;
        keypts.at(i).x = cam_matrix(0, 0) * pos_c(0) / pos_c(2) + cam_matrix(0, 2);
        keypts.at(i).y = cam_matrix(1, 1) * pos_c(1) / pos_c(2) + cam_matrix(1, 2);
    }
}

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::KeyPoint>& keypts) {
    const auto num_landmarks = landmarks.size();

    // convert a 3D point to a bearing vector

    keypts.resize(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t& pos_w = landmarks.at(i);
        Vec3_t pos_c = rot_cw * pos_w + trans_cw;
        keypts.at(i).pt.x = cam_matrix(0, 0) * pos_c(0) / pos_c(2) + cam_matrix(0, 2);
        keypts.at(i).pt.y = cam_matrix(1, 1) * pos_c(1) / pos_c(2) + cam_matrix(1, 2);
    }
}

void add_noise(std::vector<cv::Point2f>& keypts, const double noise_stddev, const double ratio) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::normal_distribution<> rand(0, noise_stddev);

    for (unsigned int i = 0; i < keypts.size() * ratio; ++i) {
        keypts.at(i) += cv::Point2f(rand(mt), rand(mt));
    }
}

void add_noise(std::vector<cv::KeyPoint>& keypts, const double noise_stddev, const double ratio) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::normal_distribution<> rand(0, noise_stddev);

    for (unsigned int i = 0; i < keypts.size() * ratio; ++i) {
        keypts.at(i).pt += cv::Point2f(rand(mt), rand(mt));
    }
}
