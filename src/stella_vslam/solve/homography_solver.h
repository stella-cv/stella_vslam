#ifndef STELLA_VSLAM_SOLVE_HOMOGRAPHY_SOLVER_H
#define STELLA_VSLAM_SOLVE_HOMOGRAPHY_SOLVER_H

#include "stella_vslam/camera/base.h"

#include <vector>
#include <random>

#include <opencv2/core/types.hpp>

namespace stella_vslam {
namespace solve {

class homography_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    homography_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                      const std::vector<std::pair<int, int>>& matches_12, const float sigma, bool use_fixed_seed = false);

    //! Destructor
    virtual ~homography_solver() = default;

    //! Find the most reliable homography matrix via RASNAC
    void find_via_ransac(const unsigned int max_num_iter, const bool recompute = true);

    //! Check if the solution is valid or not
    bool solution_is_valid() const {
        return solution_is_valid_;
    }

    //! Get the best score
    double get_best_score() const {
        return best_score_;
    }

    //! Get the most reliable essential matrix
    Mat33_t get_best_H_21() const {
        return best_H_21_;
    }

    //! Get the inlier matches
    std::vector<bool> get_inlier_matches() const {
        return is_inlier_match_;
    }

    //! Compute a homography matrix with 4-point algorithm
    static Mat33_t compute_H_21(const std::vector<cv::Point2f>& keypts_1, const std::vector<cv::Point2f>& keypts_2);

    //! Decompose a homography matrix to eight pairs of rotation and translation
    static bool decompose(const Mat33_t& H_21, const Mat33_t& cam_matrix_1, const Mat33_t& cam_matrix_2,
                          eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes, eigen_alloc_vector<Vec3_t>& init_normals);

private:
    //! Check inliers of homography transformation
    //! (Note: inlier flags are set to_inlier_match and a score is returned)
    float check_inliers(const Mat33_t& H_21, std::vector<bool>& is_inlier_match);

    //! undistorted keypoints of shot 1
    const std::vector<cv::KeyPoint> undist_keypts_1_;
    //! undistorted keypoints of shot 2
    const std::vector<cv::KeyPoint> undist_keypts_2_;
    //! matched indices between shots 1 and 2
    const std::vector<std::pair<int, int>>& matches_12_;
    //! standard deviation of keypoint detection error
    const float sigma_;

    //! solution is valid or not
    bool solution_is_valid_ = false;
    //! best score of RANSAC
    double best_score_ = 0.0;
    //! most reliable homography matrix
    Mat33_t best_H_21_;
    //! inlier matches computed via RANSAC
    std::vector<bool> is_inlier_match_;
    //! random engine for RANSAC
    std::mt19937 random_engine_;
};

} // namespace solve
} // namespace stella_vslam

#endif // STELLA_VSLAM_SOLVE_HOMOGRAPHY_SOLVER_H
