#include "stella_vslam/type.h"
#include "stella_vslam/util/trigonometric.h"

#include <gtest/gtest.h>

using namespace stella_vslam;

TEST(trigonometric, cos) {
    for (unsigned int deg = 0; deg <= 3600; ++deg) {
        const float rad = (deg / 10.0f) * M_PI / 180.0f;
        EXPECT_NEAR(std::cos(rad), util::cos(rad), 1e-3);
    }
}

TEST(trigonometric, sin) {
    for (unsigned int deg = 0; deg <= 3600; ++deg) {
        const float rad = (deg / 10.0f) * M_PI / 180.0f;
        EXPECT_NEAR(std::sin(rad), util::sin(rad), 1e-3);
    }
}
