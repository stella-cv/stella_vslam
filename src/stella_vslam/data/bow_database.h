#ifndef STELLA_VSLAM_DATA_BOW_DATABASE_H
#define STELLA_VSLAM_DATA_BOW_DATABASE_H

#include "stella_vslam/data/bow_vocabulary.h"

#include <mutex>
#include <list>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <memory>

namespace stella_vslam {
namespace data {

class frame;
class keyframe;

class bow_database {
public:
    /**
     * Constructor
     * @param bow_vocab
     */
    explicit bow_database(bow_vocabulary* bow_vocab);

    /**
     * Destructor
     */
    ~bow_database();

    /**
     * Add a keyframe to the database
     * @param keyfrm
     */
    void add_keyframe(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Erase the keyframe from the database
     * @param keyfrm
     */
    void erase_keyframe(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Clear the database
     */
    void clear();

    /**
     * Acquire keyframes over score
     */
    std::vector<std::shared_ptr<keyframe>> acquire_keyframes(const bow_vector& bow_vec, const float min_score = 0.0f,
                                                             const std::set<std::shared_ptr<keyframe>>& keyfrms_to_reject = {});

protected:
    /**
     * Initialize temporary variables
     */
    void initialize();

    /**
     * Compute the number of shared words and set candidates (init_candidates_ and num_common_words_)
     * @tparam T
     * @param bow_vec
     * @param keyfrms_to_reject
     * @return whether candidates are found or not
     */
    bool set_candidates_sharing_words(const bow_vector& bow_vec, const std::set<std::shared_ptr<keyframe>>& keyfrms_to_reject = {});

    /**
     * Compute scores (scores_) between the query and the each of keyframes contained in the database
     * @tparam T
     * @param bow_vec
     * @param min_num_common_words_thr
     * @return whether candidates are found or not
     */
    bool compute_scores(const bow_vector& bow_vec, const unsigned int min_num_common_words_thr);

    /**
     * Align scores and keyframes only which have greater scores than the minimum one
     * @param min_num_common_words_thr
     * @param min_score
     * @return whether candidates are found or not
     */
    bool align_scores_and_keyframes(const unsigned int min_num_common_words_thr, const float min_score);

    /**
     * Compute and align total scores and keyframes
     * @param min_num_common_words_thr
     * @param min_score
     * @return
     */
    float align_total_scores_and_keyframes(const unsigned int min_num_common_words_thr, const float min_score);

    //-----------------------------------------
    // BoW feature vectors

    //! mutex to access BoW database
    mutable std::mutex mtx_;
    //! BoW database
    std::unordered_map<unsigned int, std::list<std::shared_ptr<keyframe>>> keyfrms_in_node_;

    //-----------------------------------------
    // BoW vocabulary

    //! BoW vocabulary
    bow_vocabulary* bow_vocab_;

    //-----------------------------------------
    // temporary variables

    //! mutex to access temporary variables
    mutable std::mutex tmp_mtx_;

    //! initial candidates for loop or relocalization
    std::unordered_set<std::shared_ptr<keyframe>> init_candidates_;

    // key: queryとwordを共有しているキーフレーム, value: 共有word数
    //! number of shared words between the query and the each of keyframes contained in the database
    std::unordered_map<std::shared_ptr<keyframe>, unsigned int> num_common_words_;

    // key: queryとwordを共有しているキーフレーム, value: スコア
    //! similarity scores between the query and the each of keyframes contained in the database
    std::unordered_map<std::shared_ptr<keyframe>, float> scores_;

    // min_score以上のものを保存しておくvector (スコア, キーフレーム)
    //! pairs of score and keyframe which has the larger score than the minimum one
    std::vector<std::pair<float, std::shared_ptr<keyframe>>> score_keyfrm_pairs_;

    // min_score以上のものを保存しておくvector (スコア, キーフレーム)
    //! pairs of total score and keyframe which has the larger score than the minimum one
    std::vector<std::pair<float, std::shared_ptr<keyframe>>> total_score_keyfrm_pairs_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_BOW_DATABASE_H
