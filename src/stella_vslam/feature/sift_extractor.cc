#include "stella_vslam/feature/sift_extractor.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <random>
namespace stella_vslam {
namespace feature {
sift_extractor::sift_extractor(const sift_params* params)
    : extractor(), params_(params) {
}

// Generate random hyperplanes (size: 128 x num_bits)
cv::Mat generateRandomHyperplanes(int num_bits, int dim = 128) {
    cv::Mat hyperplanes(dim, num_bits, CV_32F);
    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<float> dist(0.0f, 1.0f);

    for (int i = 0; i < dim; ++i)
        for (int j = 0; j < num_bits; ++j)
            hyperplanes.at<float>(i, j) = dist(rng);

    return hyperplanes;
}

// Project SIFT descriptors using LSH and binarize
void computeBinarySIFT(const cv::_OutputArray& descriptors, const cv::Mat& hyperplanes) {
    cv::Mat desc_mat = descriptors.getMat();
    cv::Mat projections = desc_mat * hyperplanes; // [N x 128] * [128 x num_bits] = [N x num_bits]
    int num_keypoints = projections.rows;
    int num_bits = projections.cols;
    int num_bytes = (num_bits + 7) / 8;

    cv::Mat binary_descriptors(num_keypoints, num_bytes, CV_8U, cv::Scalar(0));

    for (int i = 0; i < num_keypoints; ++i) {
        for (int j = 0; j < num_bits; ++j) {
            if (projections.at<float>(i, j) > 0) {
                binary_descriptors.at<uchar>(i, j / 8) |= (1 << (7 - (j % 8)));
            }
        }
    }

    descriptors.assign(binary_descriptors);
}

void sift_extractor::extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                             std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors) {
    auto sift_detector = cv::SIFT::create(0, this->params_->num_sublevels_, this->params_->threshold_, this->params_->edge_threshold_, this->params_->scale_factor_, CV_8U);
    sift_detector->detectAndCompute(in_image, in_image_mask, keypts, out_descriptors);

    int num_bits = 256; // ORB-like
    cv::Mat hyperplanes = generateRandomHyperplanes(num_bits);
    computeBinarySIFT(out_descriptors, hyperplanes);
}
} // namespace feature
} // namespace stella_vslam