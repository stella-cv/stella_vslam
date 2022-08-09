#ifndef STELLA_VSLAM_MODULE_RELOCALIZER_H
#define STELLA_VSLAM_MODULE_RELOCALIZER_H

#include "stella_vslam/match/bow_tree.h"
#include "stella_vslam/match/projection.h"
#include "stella_vslam/match/robust.h"
#include "stella_vslam/optimize/pose_optimizer.h"
#include "stella_vslam/solve/pnp_solver.h"

#include <memory>

namespace stella_vslam {

namespace data {
class frame;
class bow_database;
} // namespace data

namespace module {

class relocalizer {
public:
    //! Constructor
    explicit relocalizer(const double bow_match_lowe_ratio = 0.75, const double proj_match_lowe_ratio = 0.9,
                         const double robust_match_lowe_ratio = 0.8,
                         const unsigned int min_num_bow_matches = 20, const unsigned int min_num_valid_obs = 50,
                         const bool use_fixed_seed = false);

    explicit relocalizer(const YAML::Node& yaml_node);

    //! Destructor
    virtual ~relocalizer();

    //! Relocalize the specified frame
    bool relocalize(data::bow_database* bow_db, data::frame& curr_frm);

    //! Relocalize the specified frame by given candidates list
    bool reloc_by_candidates(data::frame& curr_frm,
                             const std::vector<std::shared_ptr<stella_vslam::data::keyframe>>& reloc_candidates,
                             bool use_robust_matcher = false);
    bool reloc_by_candidate(data::frame& curr_frm,
                            const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                            bool use_robust_matcher);
    bool relocalize_by_pnp_solver(data::frame& curr_frm,
                                  const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                                  bool use_robust_matcher,
                                  std::vector<unsigned int>& inlier_indices,
                                  std::vector<std::shared_ptr<data::landmark>>& matched_landmarks) const;
    bool optimize_pose(data::frame& curr_frm,
                       const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                       std::vector<bool>& outlier_flags) const;
    bool refine_pose(data::frame& curr_frm,
                     const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm,
                     const std::set<std::shared_ptr<data::landmark>>& already_found_landmarks) const;
    bool refine_pose_by_local_map(data::frame& curr_frm,
                                  const std::shared_ptr<stella_vslam::data::keyframe>& candidate_keyfrm) const;

private:
    //! Extract valid (non-deleted) landmarks from landmark vector
    std::vector<unsigned int> extract_valid_indices(const std::vector<std::shared_ptr<data::landmark>>& landmarks) const;

    //! Setup PnP solver with the specified 2D-3D matches
    std::unique_ptr<solve::pnp_solver> setup_pnp_solver(const std::vector<unsigned int>& valid_indices,
                                                        const eigen_alloc_vector<Vec3_t>& bearings,
                                                        const std::vector<cv::KeyPoint>& keypts,
                                                        const std::vector<std::shared_ptr<data::landmark>>& matched_landmarks,
                                                        const std::vector<float>& scale_factors) const;

    //! minimum threshold of the number of BoW matches
    const unsigned int min_num_bow_matches_;
    //! minimum threshold of the number of valid (= inlier after pose optimization) matches
    const unsigned int min_num_valid_obs_;

    //! BoW matcher
    const match::bow_tree bow_matcher_;
    //! projection matcher
    const match::projection proj_matcher_;
    //! robust matcher
    const match::robust robust_matcher_;
    //! pose optimizer
    const optimize::pose_optimizer pose_optimizer_;

    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_RELOCALIZER_H
