#ifndef STELLA_VSLAM_SOLVE_ESSENTIAL_SOLVER_H
#define STELLA_VSLAM_SOLVE_ESSENTIAL_SOLVER_H

#include "stella_vslam/type.h"

#include <vector>
#include <random>

namespace stella_vslam {
namespace solve {

class essential_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    essential_solver(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2,
                     const std::vector<std::pair<int, int>>& matches_12, bool use_fixed_seed = false);

    //! Destructor
    virtual ~essential_solver() = default;

    //! Find the most reliable essential matrix via RANSAC
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
    Mat33_t get_best_E_21() const {
        return best_E_21_;
    }

    //! Get the inlier matches
    std::vector<bool> get_inlier_matches() const {
        return is_inlier_match_;
    }

    //! Compute an essential matrix with 8-point algorithm
    static Mat33_t compute_E_21(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2);

    //! Decompose an essential matrix to four pairs of rotation and translation
    static bool decompose(const Mat33_t& E_21, eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes);

    //! Create an essential matrix from camera poses
    static Mat33_t create_E_21(const Mat33_t& rot_1w, const Vec3_t& trans_1w, const Mat33_t& rot_2w, const Vec3_t& trans_2w);

private:
    //! Check inliers of the epipolar constraint
    //! (Note: inlier flags are set to `inlier_match` and a score is returned)
    float check_inliers(const Mat33_t& E_21, std::vector<bool>& is_inlier_match);

    //! bearing vectors of shot 1
    const eigen_alloc_vector<Vec3_t>& bearings_1_;
    //! bearing vectors of shot 2
    const eigen_alloc_vector<Vec3_t>& bearings_2_;
    //! matched indices between shots 1 and 2
    const std::vector<std::pair<int, int>>& matches_12_;

    //! solution is valid or not
    bool solution_is_valid_ = false;
    //! best score of RANSAC
    double best_score_ = 0.0;
    //! most reliable essential matrix
    Mat33_t best_E_21_;
    //! inlier matches computed via RANSAC
    std::vector<bool> is_inlier_match_;
    //! random engine for RANSAC
    std::mt19937 random_engine_;
};

} // namespace solve
} // namespace stella_vslam

#endif // STELLA_VSLAM_SOLVE_ESSENTIAL_SOLVER_H
