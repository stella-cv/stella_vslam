#include "stella_vslam/type.h"
#include "stella_vslam/match/base.h"

#include <chrono>
#include <cmath>
#include <thread>

#include <gtest/gtest.h>

using namespace stella_vslam;

TEST(base, compute_hamming_distance_1) {
    cv::Mat desc_1(1, 32, CV_8U);
    cv::Mat desc_2(1, 32, CV_8U);

    for (int i = 0; i < desc_1.rows; ++i) {
        desc_1.row(i) = 0b01010101;
    }

    for (int i = 0; i < desc_2.rows; ++i) {
        desc_2.row(i) = 0b01010101;
    }

    EXPECT_EQ(match::compute_descriptor_distance(desc_1, desc_2), 0);
}

TEST(base, compute_hamming_distance_2) {
    cv::Mat desc_1(1, 32, CV_8U);
    cv::Mat desc_2(1, 32, CV_8U);

    for (int i = 0; i < desc_1.rows; ++i) {
        desc_1.row(i) = 0b01010101;
    }

    for (int i = 0; i < desc_2.rows; ++i) {
        desc_2.row(i) = 0b10101010;
    }

    EXPECT_EQ(match::compute_descriptor_distance(desc_1, desc_2), 256);
}

TEST(base, compute_hamming_distance_3) {
    cv::Mat desc_1(1, 32, CV_8U);
    cv::Mat desc_2(1, 32, CV_8U);

    for (int i = 0; i < desc_1.rows; ++i) {
        desc_1.row(i) = 0b01100110;
    }

    for (int i = 0; i < desc_2.rows; ++i) {
        desc_2.row(i) = 0b00111100;
    }

    EXPECT_EQ(match::compute_descriptor_distance(desc_1, desc_2), 128);
}

TEST(base, compute_descriptor_distance_float_1) {
    // Identical normalized vectors → L2 = 0
    cv::Mat desc_1(1, 32, CV_32F);
    cv::Mat desc_2(1, 32, CV_32F);
    desc_1.setTo(1.0f / std::sqrt(32.0f));
    desc_2.setTo(1.0f / std::sqrt(32.0f));
    EXPECT_EQ(match::compute_descriptor_distance(desc_1, desc_2), 0);
}

TEST(base, compute_descriptor_distance_float_2) {
    // Opposite unit vectors → L2 = 2.0 → scaled = 256 (capped)
    cv::Mat desc_1(1, 32, CV_32F);
    cv::Mat desc_2(1, 32, CV_32F);
    desc_1.setTo(1.0f / std::sqrt(32.0f));
    desc_2.setTo(-1.0f / std::sqrt(32.0f));
    EXPECT_GE(match::compute_descriptor_distance(desc_1, desc_2), 255);
}

TEST(base, compute_descriptor_distance_float_3) {
    // Orthogonal unit vectors → L2 = sqrt(2) ≈ 1.414 → scaled = 181
    // [1,0,...] and [0,1,...] are already unit vectors (L2 norm = 1)
    cv::Mat desc_1 = cv::Mat::zeros(1, 32, CV_32F);
    cv::Mat desc_2 = cv::Mat::zeros(1, 32, CV_32F);
    desc_1.at<float>(0, 0) = 1.0f;
    desc_2.at<float>(0, 1) = 1.0f;
    EXPECT_EQ(match::compute_descriptor_distance(desc_1, desc_2), 181);
}
