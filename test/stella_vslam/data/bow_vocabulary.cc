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
    auto extractor = feature::orb_extractor(&params, 1000, {{0.0, 0.2, 0.0, 1.0}, {0.8, 1.0, 0.0, 1.0}});

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

#ifdef USE_DBOW2
    bow_vocab->transform(util::converter::to_desc_vec(desc1), bow_vec_1, bow_feat_vec_1, 4);
    bow_vocab->transform(util::converter::to_desc_vec(desc2), bow_vec_2, bow_feat_vec_2, 4);
#else
    bow_vocab->transform(desc1, 4, bow_vec_1, bow_feat_vec_1);
    bow_vocab->transform(desc2, 4, bow_vec_2, bow_feat_vec_2);
#endif

#ifdef USE_DBOW2
    const float score = bow_vocab->score(bow_vec_1, bow_vec_2);
#else
    const float score = fbow::BoWVector::score(bow_vec_1, bow_vec_2);
#endif
    return score;
}

TEST(bow_vocabulary, match_near_scene) {
    const auto vocab_file_path_env = std::getenv("BOW_VOCAB");
    const std::string vocab_file_path = (vocab_file_path_env != nullptr) ? vocab_file_path_env : "";
    if (vocab_file_path.empty()) {
        return;
    }

#ifdef USE_DBOW2
    auto bow_vocab = new data::bow_vocabulary();
    try {
        bow_vocab->loadFromBinaryFile(vocab_file_path);
    }
    catch (const std::exception&) {
        std::cerr << "wrong path to vocabulary" << std::endl;
        delete bow_vocab;
        bow_vocab = nullptr;
        exit(EXIT_FAILURE);
    }
#else
    auto bow_vocab = new fbow::Vocabulary();
    bow_vocab->readFromFile(vocab_file_path);
    if (!bow_vocab->isValid()) {
        std::cerr << "wrong path to vocabulary" << std::endl;
        delete bow_vocab;
        bow_vocab = nullptr;
        exit(EXIT_FAILURE);
    }
#endif

    auto score = get_score(bow_vocab,
                           std::string(TEST_DATA_DIR) + "./equirectangular_image_001.jpg",
                           std::string(TEST_DATA_DIR) + "./equirectangular_image_002.jpg");
    EXPECT_LT(score, 1.0);

    delete bow_vocab;
}
