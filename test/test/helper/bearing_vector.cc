#include "helper/bearing_vector.h"
#include "stella_vslam/util/converter.h"

#include <random>

void create_bearing_vectors(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const eigen_alloc_vector<Vec3_t>& landmarks,
                            eigen_alloc_vector<Vec3_t>& bearings) {
    const auto num_landmarks = landmarks.size();

    // convert a 3D point to a bearing vector
    bearings.resize(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t& lm = landmarks.at(i);
        const Vec3_t lm_in_1 = rot_cw * lm + trans_cw;
        bearings.at(i) = lm_in_1.normalized();
    }
}

void add_noise(eigen_alloc_vector<Vec3_t>& bearings, const double noise_stddev, const double ratio) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::normal_distribution<> rand_angle(0, noise_stddev);
    std::uniform_real_distribution<> rand_axis(-1.0, 1.0);

    for (unsigned int i = 0; i < bearings.size() * ratio; ++i) {
        Vec3_t angle_axis{rand_axis(mt), rand_axis(mt), rand_axis(mt)};
        Mat33_t rot = util::converter::to_rot_mat(angle_axis.normalized() * rand_angle(mt));
        bearings.at(i) = rot * bearings.at(i);
    }
}
