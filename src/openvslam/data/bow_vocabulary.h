#ifndef OPENVSLAM_DATA_BOW_VOCABULARY_H
#define OPENVSLAM_DATA_BOW_VOCABULARY_H

#include "openvslam/data/bow_vocabulary_fwd.h"

#ifdef USE_DBOW2
#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>
#else
#include <fbow/vocabulary.h>
#endif // USE_DBOW2

#endif // OPENVSLAM_DATA_BOW_VOCABULARY_H
