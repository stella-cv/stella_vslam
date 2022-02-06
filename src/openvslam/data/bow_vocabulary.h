#ifndef OPENVSLAM_DATA_BOW_VOCABULARY_H
#define OPENVSLAM_DATA_BOW_VOCABULARY_H

#include "openvslam/data/bow_vocabulary_fwd.h"

#ifdef USE_DBOW2
#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>
#else
#include <fbow/vocabulary.h>
#endif // USE_DBOW2

namespace openvslam {
namespace data {
namespace bow_vocabulary_util {

void compute_bow(data::bow_vocabulary* bow_vocab, const cv::Mat& descriptors, data::bow_vector& bow_vec, data::bow_feature_vector& bow_feat_vec);

}; // namespace bow_vocabulary_util
}; // namespace data
}; // namespace openvslam

#endif // OPENVSLAM_DATA_BOW_VOCABULARY_H
