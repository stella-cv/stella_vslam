#include "helper/landmark.h"

eigen_alloc_vector<Vec3_t> create_random_landmarks_in_space(const unsigned int num_landmarks,
                                                            const float space_lim) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::uniform_real_distribution<> rand(-space_lim, space_lim);

    eigen_alloc_vector<Vec3_t> landmarks(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        landmarks.at(i)(0) = rand(mt);
        landmarks.at(i)(1) = rand(mt);
        landmarks.at(i)(2) = rand(mt);
    }

    return landmarks;
}

eigen_alloc_vector<Vec3_t> create_random_landmarks_on_plane(const unsigned int num_landmarks,
                                                            const float space_lim, const Vec4_t& plane_coeffs) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::uniform_real_distribution<> rand(-space_lim, space_lim);

    const auto a = plane_coeffs(0);
    const auto b = plane_coeffs(1);
    const auto c = plane_coeffs(2);
    const auto d = plane_coeffs(3);

    eigen_alloc_vector<Vec3_t> landmarks(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const double x = rand(mt);
        const double y = rand(mt);
        const double z = -(d + a * x + b * y) / c;
        if (z < -space_lim || space_lim < z) {
            --i;
            continue;
        }
        landmarks.at(i)(0) = x;
        landmarks.at(i)(1) = y;
        landmarks.at(i)(2) = z;
    }

    return landmarks;
}

eigen_alloc_vector<Vec3_t> create_landmarks_on_plane(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix,
                                                     const std::vector<cv::Point2f>& keypts, const Vec4_t& plane_coeffs) {
    const auto a = plane_coeffs(0);
    const auto b = plane_coeffs(1);
    const auto c = plane_coeffs(2);
    const auto d = plane_coeffs(3);

    Vec3_t n_w(a, b, c);
    Vec3_t n_c = rot_cw * n_w;
    Vec3_t p_w(0, 0, -d / c);
    Vec3_t p_c = rot_cw * p_w + trans_cw;

    eigen_alloc_vector<Vec3_t> landmarks(keypts.size());
    for (unsigned int i = 0; i < keypts.size(); ++i) {
        Vec3_t pos_c;
        double u = (keypts.at(i).x - cam_matrix(0, 2)) / cam_matrix(0, 0);
        double v = (keypts.at(i).y - cam_matrix(1, 2)) / cam_matrix(1, 1);
        pos_c(2) = (p_c(2) + n_c(0) / n_c(2) * p_c(0) + n_c(1) / n_c(2) * p_c(1)) / (1 + n_c(1) / n_c(2) * v + n_c(0) / n_c(2) * u);
        pos_c(0) = pos_c(2) * u;
        pos_c(1) = pos_c(2) * v;

        landmarks.at(i) = rot_cw.transpose() * (pos_c - trans_cw);
    }

    return landmarks;
}

eigen_alloc_vector<Vec3_t> create_landmarks_on_plane(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix,
                                                     const std::vector<cv::KeyPoint>& keypts, const Vec4_t& plane_coeffs) {
    const auto a = plane_coeffs(0);
    const auto b = plane_coeffs(1);
    const auto c = plane_coeffs(2);
    const auto d = plane_coeffs(3);

    Vec3_t n_w(a, b, c);
    Vec3_t n_c = rot_cw * n_w;
    Vec3_t p_w(0, 0, -d / c);
    Vec3_t p_c = rot_cw * p_w + trans_cw;

    eigen_alloc_vector<Vec3_t> landmarks(keypts.size());
    for (unsigned int i = 0; i < keypts.size(); ++i) {
        Vec3_t pos_c;
        double u = (keypts.at(i).pt.x - cam_matrix(0, 2)) / cam_matrix(0, 0);
        double v = (keypts.at(i).pt.y - cam_matrix(1, 2)) / cam_matrix(1, 1);
        pos_c(2) = (p_c(2) + n_c(0) / n_c(2) * p_c(0) + n_c(1) / n_c(2) * p_c(1)) / (1 + n_c(1) / n_c(2) * v + n_c(0) / n_c(2) * u);
        pos_c(0) = pos_c(2) * u;
        pos_c(1) = pos_c(2) * v;

        landmarks.at(i) = rot_cw.transpose() * (pos_c - trans_cw);
    }

    return landmarks;
}
