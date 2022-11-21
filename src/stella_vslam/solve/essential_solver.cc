#include "stella_vslam/solve/essential_solver.h"
#include "stella_vslam/util/converter.h"
#include "stella_vslam/util/random_array.h"
#include "stella_vslam/util/trigonometric.h"

namespace stella_vslam {
namespace solve {

essential_solver::essential_solver(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2,
                                   const std::vector<std::pair<int, int>>& matches_12, bool use_fixed_seed)
    : bearings_1_(bearings_1), bearings_2_(bearings_2), matches_12_(matches_12),
      random_engine_(util::create_random_engine(use_fixed_seed)) {}

void essential_solver::find_via_ransac(const unsigned int max_num_iter, const bool recompute) {
    const auto num_matches = static_cast<unsigned int>(matches_12_.size());

    // 1. Prepare for RANSAC

    // minimum number of samples (= 8)
    constexpr unsigned int min_set_size = 8;
    if (num_matches < min_set_size) {
        solution_is_valid_ = false;
        return;
    }

    // RANSAC variables
    best_cost_ = std::numeric_limits<float>::max();
    is_inlier_match_ = std::vector<bool>(num_matches, false);

    // minimum set of keypoint matches
    eigen_alloc_vector<Vec3_t> min_set_bearings_1(min_set_size);
    eigen_alloc_vector<Vec3_t> min_set_bearings_2(min_set_size);

    // shared variables in RANSAC loop
    // essential matrix from shot 1 to shot 2
    Mat33_t E_21_in_sac;
    // inlier/outlier flags
    std::vector<bool> is_inlier_match_in_sac(num_matches, false);

    // 2. RANSAC loop

    for (unsigned int iter = 0; iter < max_num_iter; iter++) {
        // 2-1. Create a minimum set
        const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1, random_engine_);
        for (unsigned int i = 0; i < min_set_size; ++i) {
            const auto idx = indices.at(i);
            min_set_bearings_1.at(i) = bearings_1_.at(matches_12_.at(idx).first);
            min_set_bearings_2.at(i) = bearings_2_.at(matches_12_.at(idx).second);
        }

        // 2-2. Compute an essential matrix
        E_21_in_sac = compute_E_21(min_set_bearings_1, min_set_bearings_2);

        // 2-3. Check inliers and compute a cost
        float cost_in_sac;
        unsigned int num_inliers = check_inliers(E_21_in_sac, is_inlier_match_in_sac, cost_in_sac);

        // 2-4. Update the best model
        if (num_inliers > min_set_size && best_cost_ > cost_in_sac) {
            best_cost_ = cost_in_sac;
            best_E_21_ = E_21_in_sac;
            is_inlier_match_ = is_inlier_match_in_sac;
        }
    }

    solution_is_valid_ = best_cost_ < std::numeric_limits<float>::max();

    if (!recompute || !solution_is_valid_) {
        return;
    }

    // 3. Recompute an essential matrix only with the inlier matches

    eigen_alloc_vector<Vec3_t> inlier_bearing_1;
    eigen_alloc_vector<Vec3_t> inlier_bearing_2;
    inlier_bearing_1.reserve(matches_12_.size());
    inlier_bearing_2.reserve(matches_12_.size());
    for (unsigned int i = 0; i < matches_12_.size(); ++i) {
        if (is_inlier_match_.at(i)) {
            inlier_bearing_1.push_back(bearings_1_.at(matches_12_.at(i).first));
            inlier_bearing_2.push_back(bearings_2_.at(matches_12_.at(i).second));
        }
    }
    best_E_21_ = solve::essential_solver::compute_E_21(inlier_bearing_1, inlier_bearing_2);
    check_inliers(best_E_21_, is_inlier_match_, best_cost_);
}

Mat33_t essential_solver::compute_E_21(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2) {
    assert(bearings_1.size() == bearings_2.size());

    const auto num_points = bearings_1.size();

    typedef Eigen::Matrix<Mat33_t::Scalar, Eigen::Dynamic, 9> CoeffMatrix;
    CoeffMatrix A(num_points, 9);

    for (unsigned int i = 0; i < num_points; i++) {
        A.block<1, 3>(i, 0) = bearings_2.at(i)(0) * bearings_1.at(i);
        A.block<1, 3>(i, 3) = bearings_2.at(i)(1) * bearings_1.at(i);
        A.block<1, 3>(i, 6) = bearings_2.at(i)(2) * bearings_1.at(i);
    }

    const Eigen::JacobiSVD<CoeffMatrix> init_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix<Mat33_t::Scalar, 9, 1> v = init_svd.matrixV().col(8);
    // need transpose() because elements are contained as col-major after it was constructed from a pointer
    const Mat33_t init_E_21 = Mat33_t(v.data()).transpose();

    const Eigen::JacobiSVD<Mat33_t> svd(init_E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Mat33_t& U = svd.matrixU();
    Vec3_t lambda = svd.singularValues();
    const Mat33_t& V = svd.matrixV();

    lambda(2) = 0.0;

    const Mat33_t E_21 = U * lambda.asDiagonal() * V.transpose();

    return E_21;
}

bool essential_solver::decompose(const Mat33_t& E_21, eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes) {
    // https://en.wikipedia.org/wiki/Essential_matrix#Determining_R_and_t_from_E

    const Eigen::JacobiSVD<Mat33_t> svd(E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Vec3_t trans = svd.matrixU().col(2);
    trans.normalize();

    Mat33_t W = Mat33_t::Zero();
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;

    Mat33_t rot_1 = svd.matrixU() * W * svd.matrixV().transpose();
    if (rot_1.determinant() < 0) {
        rot_1 *= -1;
    }

    Mat33_t rot_2 = svd.matrixU() * W.transpose() * svd.matrixV().transpose();
    if (rot_2.determinant() < 0) {
        rot_2 *= -1;
    }

    init_rots = {rot_1, rot_1, rot_2, rot_2};
    init_transes = {trans, -trans, trans, -trans};

    return true;
}

Mat33_t essential_solver::create_E_21(const Mat33_t& rot_1w, const Vec3_t& trans_1w, const Mat33_t& rot_2w, const Vec3_t& trans_2w) {
    const Mat33_t rot_21 = rot_2w * rot_1w.transpose();
    const Vec3_t trans_21 = -rot_21 * trans_1w + trans_2w;
    const Mat33_t trans_21_x = util::converter::to_skew_symmetric_mat(trans_21);
    return trans_21_x * rot_21;
}

unsigned int essential_solver::check_inliers(const Mat33_t& E_21, std::vector<bool>& is_inlier_match, float& cost) {
    unsigned int num_inliers = 0;
    const auto num_points = matches_12_.size();

    is_inlier_match.resize(num_points);

    const Mat33_t E_12 = E_21.transpose();

    cost = 0.0;

    // outlier threshold of cosine between a bearing vector and the epipolar plane
    const float cos_angle_thr = util::cos(1.0 * M_PI / 180.0);

    for (unsigned int i = 0; i < num_points; ++i) {
        const auto& bearing_1 = bearings_1_.at(matches_12_.at(i).first);
        const auto& bearing_2 = bearings_2_.at(matches_12_.at(i).second);

        const Vec3_t epiplane_in_2 = E_21 * bearing_1;
        const float cos_in_2 = epiplane_in_2.cross(bearing_2).norm() / epiplane_in_2.norm();

        const Vec3_t epiplane_in_1 = E_12 * bearing_2;
        const float cos_in_1 = epiplane_in_1.cross(bearing_1).norm() / epiplane_in_1.norm();

        float worst_cos_angle = std::min(cos_in_1, cos_in_2);

        if (cos_angle_thr < worst_cos_angle) {
            is_inlier_match.at(i) = true;
            cost += 1.0 - worst_cos_angle;
            num_inliers++;
        }
        else {
            is_inlier_match.at(i) = false;
            cost += 1.0 - cos_angle_thr;
        }
    }

    return num_inliers;
}

} // namespace solve
} // namespace stella_vslam
