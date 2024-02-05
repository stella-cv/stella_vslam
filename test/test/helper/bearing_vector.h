#ifndef STELLA_VSLAM_TEST_HELPER_BEARING_VECTOR_H
#define STELLA_VSLAM_TEST_HELPER_BEARING_VECTOR_H

#include "stella_vslam/type.h"

using namespace stella_vslam;

void create_bearing_vectors(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const eigen_alloc_vector<Vec3_t>& landmarks,
                            eigen_alloc_vector<Vec3_t>& bearings);

void add_noise(eigen_alloc_vector<Vec3_t>& bearings, const double noise_stddev, const double ratio);

#endif // STELLA_VSLAM_TEST_HELPER_BEARING_VECTOR_H
