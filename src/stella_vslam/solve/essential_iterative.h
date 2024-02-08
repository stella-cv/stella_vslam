/**
 *
 * Implementation of Algorithm 3 from: Improved RANSAC performance using simple, iterative minimal-set solvers
 * https://arxiv.org/abs/1007.1432
 *
 */

#ifndef STELLA_VSLAM_SOLVE_MINIMAL_ESSENTIAL_SOLVER_H
#define STELLA_VSLAM_SOLVE_MINIMAL_ESSENTIAL_SOLVER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>

#include "stella_vslam/type.h"

namespace minimal_solver {

using namespace Eigen;

// Initialize SO(3) generators
const Matrix3d G1 = (Matrix3d() << 0, 1, 0,
                     -1, 0, 0,
                     0, 0, 0)
                        .finished();
const Matrix3d G2 = (Matrix3d() << 0, 0, 1,
                     0, 0, 0,
                     -1, 0, 0)
                        .finished();
const Matrix3d G3 = (Matrix3d() << 0, 0, 0,
                     0, 0, 1,
                     0, -1, 0)
                        .finished();
const Vector3d tr = (Vector3d() << 1, 0, 0).finished();
const Matrix<double, 2, 3> P = (Matrix<double, 2, 3>() << 1, 0, 0,
                               0, 1, 0)
                                  .finished();

Matrix3d skew(const Array3d& x) {
    Matrix3d result;
    result << 0, -x[2], x[1],
        x[2], 0, -x[0],
        -x[1], x[0], 0;
    return result;
}

Matrix3d taylor_expm(const Matrix<double, 3, 3>& A) {
    Matrix3d result = Matrix3d::Identity(); // Initialize result as the identity matrix
    Matrix3d term = Matrix3d::Identity();   // Initialize the first term in the series as the identity matrix

    constexpr int n_terms = 10;
    for (int i = 1; i < n_terms; ++i) {
        term = term * A / i; // Update the term using matrix multiplication and division
        result += term;
    }
    return result;
}

std::pair<Matrix<double, Dynamic, 5>, Matrix<double, Dynamic, 1>> calc_Jr(const Matrix<double, 5, 1>& a,
                                                            const Matrix<double, Dynamic, 3>& q1,
                                                            const Matrix<double, Dynamic, 3>& q2) {
    const Matrix3d R = taylor_expm(a[0] * G1 + a[1] * G2 + a[2] * G3);
    const Matrix3d Rt = taylor_expm(a[3] * G1 + a[4] * G2);

    const Vector3d tvec = Rt * tr;
    const Matrix3d tx = skew(tvec);

    const Matrix3d E = tx * R;

    // Residual
    Matrix<double, 5, 1> r = (q2 * E * q1.transpose()).diagonal();

    // Jacobian
    MatrixXd J = MatrixXd::Zero(q1.rows(), 5);
    const auto q2tx = q2 * tx;
    const auto Rq1 = R * q1.transpose();
    const Matrix3d tmp4 = skew((Rt * G1 * tr).array());
    const Matrix3d tmp5 = skew((Rt * G2 * tr).array());

    for (int i = 0; i < q1.rows(); ++i) {
        J(i, 0) = q2tx.row(i) * G1 * Rq1.col(i);
        J(i, 1) = q2tx.row(i) * G2 * Rq1.col(i);
        J(i, 2) = q2tx.row(i) * G3 * Rq1.col(i);
        J(i, 3) = q2.row(i) * tmp4 * Rq1.col(i);
        J(i, 4) = q2.row(i) * tmp5 * Rq1.col(i);
    }

    return {J, r};
}

Matrix3d computeE_iterative_lm(const Eigen::Matrix<double, Dynamic, 3>& q1,
                      const Eigen::Matrix<double, Dynamic, 3>& q2,
                      Matrix<double, 5, 1>& a) {

    // Perform N iterations of LM (we do not need tight convergence yet)
    const int max_iter = 200;
    const double tol = 0.001;
    double damping_factor = 0.1; // Damping factor for LM
    int iter = 0;
    Matrix<double, Dynamic, 1> last_r;

    for (iter = 0; iter < max_iter; ++iter) {
        // TODO: Remove c++17 feature
        auto [J, r] = calc_Jr(a, q1, q2);

        // LM update step
        MatrixXd A = J.transpose() * J;
        MatrixXd A_diag = A.diagonal().asDiagonal();
        VectorXd b = J.transpose() * r;
        VectorXd delta = (A + damping_factor * A_diag).ldlt().solve(b);

        a -= delta;

        if (r.norm() < tol) {
            break;
        }
    }

    // TODO: What if it didnt converge? LM should only be used to refine E based on inlier's though
    const Matrix3d R = taylor_expm(a[0] * G1 + a[1] * G2 + a[2] * G3);
    const Matrix3d Rt = taylor_expm(a[3] * G1 + a[4] * G2);

    const Vector3d tvec = Rt * tr;
    const Matrix3d tx = skew(tvec);

    Matrix3d E = tx * R;
    
    return E;
}

Matrix3d computeE_iterative_gn(const Eigen::Matrix<double, Dynamic, 3>& q1,
                      const Eigen::Matrix<double, Dynamic, 3>& q2,
                      Matrix<double, 5, 1>& a) {

    // Perform N iterations of Gauss Newton (we do not need tight convergence yet)
    const int max_iter = 10;
    const double tol = 0.001;
    int iter = 0;
    Matrix<double, Dynamic, 1> last_r;

    for (iter = 0; iter < max_iter; ++iter) {
        // TODO: Remove c++17 feature
        auto [J, r] = calc_Jr(a, q1, q2);

        a -= J.fullPivLu().solve(r);

        double error_delta = 1;
        if (last_r.size() > 0) {
            error_delta = (r - last_r).norm();
        }
        last_r = r;

        if (error_delta < tol) {
            break;
        }
    }

    // Check if the sample converged to a reasonable residual (if not, outlier pollution is likely)
    Matrix3d E = Matrix3d::Zero();
    if (last_r.norm() <= tol) {
        // GN converged!
        const Matrix3d R = taylor_expm(a[0] * G1 + a[1] * G2 + a[2] * G3);
        const Matrix3d Rt = taylor_expm(a[3] * G1 + a[4] * G2);

        const Vector3d tvec = Rt * tr;
        const Matrix3d tx = skew(tvec);

        E = tx * R;
    }

    return E;
}

} // namespace minimal_solver

#endif // STELLA_VSLAM_SOLVE_MINIMAL_ESSENTIAL_SOLVER_H
