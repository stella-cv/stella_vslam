#include "stella_vslam/data/bow_vocabulary.h"
#include "stella_vslam/util/converter.h"
#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace data {
namespace bow_vocabulary_util {

float score(bow_vocabulary* bow_vocab, const bow_vector& bow_vec1, const bow_vector& bow_vec2) {
#ifdef USE_DBOW2
    return bow_vocab->score(bow_vec1, bow_vec2);
#else
    (void)bow_vocab;
    return fbow::BoWVector::score(bow_vec1, bow_vec2);
#endif
}

void compute_bow(bow_vocabulary* bow_vocab, const cv::Mat& descriptors, bow_vector& bow_vec, bow_feature_vector& bow_feat_vec) {
#ifdef USE_DBOW2
    bow_vocab->transform(util::converter::to_desc_vec(descriptors), bow_vec, bow_feat_vec, 4);
#else
    bow_vocab->transform(descriptors, 4, bow_vec, bow_feat_vec);
#endif
}

bow_vocabulary* load(std::string path) {
    bow_vocabulary* bow_vocab = new bow_vocabulary();
#ifdef USE_DBOW2
    try {
        bow_vocab->loadFromBinaryFile(path);
    }
    catch (const std::exception&) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab;
        bow_vocab = nullptr;
        exit(EXIT_FAILURE);
    }
#else
    bow_vocab->readFromFile(path);
    if (!bow_vocab->isValid()) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab;
        bow_vocab = nullptr;
        exit(EXIT_FAILURE);
    }
#endif
    return bow_vocab;
}

}; // namespace bow_vocabulary_util
}; // namespace data
}; // namespace stella_vslam
