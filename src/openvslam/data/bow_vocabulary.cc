#include "openvslam/data/bow_vocabulary.h"

namespace openvslam {
namespace data {
namespace bow_vocabulary_util {

void compute_bow(data::bow_vocabulary* bow_vocab, const cv::Mat& descriptors, data::bow_vector& bow_vec, data::bow_feature_vector& bow_feat_vec) {
#ifdef USE_DBOW2
    bow_vocab->transform(util::converter::to_desc_vec(descriptors), bow_vec, bow_feat_vec, 4);
#else
    bow_vocab->transform(descriptors, 4, bow_vec, bow_feat_vec);
#endif
}

}; // namespace bow_vocabulary_util
}; // namespace data
}; // namespace openvslam
