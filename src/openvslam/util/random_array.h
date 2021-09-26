#ifndef OPENVSLAM_UTIL_RANDOM_ARRAY_H
#define OPENVSLAM_UTIL_RANDOM_ARRAY_H

#include "openvslam/platform.h"

#include <vector>
#include <random>
#include <memory>

namespace openvslam {
namespace util {

// Create random_engine. If use_fixed_seed is true, a fixed seed value is used.
std::mt19937 create_random_engine(bool use_fixed_seed = false);

template<typename T>
std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max,
                                   std::mt19937& random_engine);

} // namespace util
} // namespace openvslam

#endif // OPENVSLAM_UTIL_RANDOM_ARRAY_H
