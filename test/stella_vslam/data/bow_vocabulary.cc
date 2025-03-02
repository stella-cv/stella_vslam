#include "stella_vslam/feature/orb_extractor.h"
#include "stella_vslam/data/bow_vocabulary.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <gtest/gtest.h>

using namespace stella_vslam;

float get_score(data::bow_vocabulary* bow_vocab, const std::string& file1, const std::string& file2) {
    auto params = feature::orb_params("ORB setting for test");
    // mask (Mask the top and bottom 20%)
    auto extractor = feature::orb_extractor(&params, 1000, feature::descriptor_type::ORB, {{0.0, 0.2, 0.0, 1.0}, {0.8, 1.0, 0.0, 1.0}});

    // image
    const auto img1 = cv::imread(file1, cv::IMREAD_GRAYSCALE);
    const auto img2 = cv::imread(file2, cv::IMREAD_GRAYSCALE);
    // mask (disable in_image_mask)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts1, keypts2;
    cv::Mat desc1, desc2;
    extractor.extract(img1, mask, keypts1, desc1);
    extractor.extract(img2, mask, keypts2, desc2);

    data::bow_vector bow_vec_1, bow_vec_2;
    data::bow_feature_vector bow_feat_vec_1, bow_feat_vec_2;

    data::bow_vocabulary_util::compute_bow(bow_vocab, desc1, bow_vec_1, bow_feat_vec_1);
    data::bow_vocabulary_util::compute_bow(bow_vocab, desc2, bow_vec_2, bow_feat_vec_2);

    const auto score = data::bow_vocabulary_util::score(bow_vocab, bow_vec_1, bow_vec_2);
    return score;
}

TEST(bow_vocabulary, match_near_scene) {
    const auto vocab_file_path_env = std::getenv("BOW_VOCAB");
    const std::string vocab_file_path = (vocab_file_path_env != nullptr) ? vocab_file_path_env : "";
    if (vocab_file_path.empty()) {
        return;
    }

    auto bow_vocab = data::bow_vocabulary_util::load(vocab_file_path);

    auto score = get_score(bow_vocab,
                           std::string(TEST_DATA_DIR) + "./equirectangular_image_001.jpg",
                           std::string(TEST_DATA_DIR) + "./equirectangular_image_002.jpg");
    EXPECT_LT(score, 1.0);

    delete bow_vocab;
}
