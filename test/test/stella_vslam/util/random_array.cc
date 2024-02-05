#include "stella_vslam/type.h"
#include "stella_vslam/util/random_array.h"

#include <gtest/gtest.h>

using namespace stella_vslam;

TEST(random_array, create_random_array_1) {
    auto random_engine = util::create_random_engine();
    const auto array = util::create_random_array(5, 1, 5, random_engine);

    EXPECT_EQ(array.size(), 5);
    // We are creating a random array with no duplicates, so the minimum value should be 1 and the maximum value should be 5.
    EXPECT_EQ(*std::min_element(array.begin(), array.end()), 1);
    EXPECT_EQ(*std::max_element(array.begin(), array.end()), 5);
}

TEST(random_array, create_random_array_2) {
    auto random_engine = util::create_random_engine();
    const auto array = util::create_random_array(10, 2, 11, random_engine);

    EXPECT_EQ(array.size(), 10);
    // Since we are creating a random array with no duplicates, the size should not change even if we convert it to std::set
    EXPECT_EQ(array.size(), std::set<int>(array.begin(), array.end()).size());
}
