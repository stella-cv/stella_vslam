#ifndef STELLA_VSLAM_SOLVE_ESSENTIAL_5PT_H
#define STELLA_VSLAM_SOLVE_ESSENTIAL_5PT_H

#include <Eigen/Dense>

#include "stella_vslam/type.h"

/**
 * This file contains helper functions to compute the essential matrix from 5 bearing vector correspondences.
 * The code is adapted from the OpenMVG and TheiaSfm implementations of Stewenius's et. al's algorithm from
 * "Recent developments on direct relative orientation".
 * 
 */

namespace stella_vslam {

// Polynomial coefficients
// slightly different from the paper, which seems to have a typo here
enum {
    poly_xxx,
    poly_xxy,
    poly_xyy,
    poly_yyy,
    poly_xxz,
    poly_xyz,
    poly_yyz,
    poly_xzz,
    poly_yzz,
    poly_zzz,
    poly_xx,
    poly_xy,
    poly_yy,
    poly_xz,
    poly_yz,
    poly_zz,
    poly_x,
    poly_y,
    poly_z,
    poly_1
};

MatX_t find_nullspace_of_epipolar_constraint(const eigen_alloc_vector<Vec3_t>& x1, const eigen_alloc_vector<Vec3_t>& x2, bool& success) {
    Eigen::Matrix<double, 9, 9> epipolar_constraint = Eigen::Matrix<double, 9, 9>::Constant(0.0);
    // form the epipolar constraint from the bearing vectors
    for (size_t i = 0; i < x1.size(); ++i) {
        epipolar_constraint.row(i) << x2.at(i)(0) * x1.at(i).transpose(),
            x2.at(i)(1) * x1.at(i).transpose(),
            x2.at(i)(2) * x1.at(i).transpose();
    }

    // Extract the null space from a minimal sampling (using LU) or non-minimal
    // sampling (using SVD).
    success = false;
    MatX_t null_space;
    if (x1.size() == 5) {
        const Eigen::FullPivLU<MatX_t> lu(epipolar_constraint);
        success = (lu.dimensionOfKernel() >= 4);
        null_space = lu.kernel();
    }
    else {
        const Eigen::JacobiSVD<MatX_t> svd(
            epipolar_constraint.transpose() * epipolar_constraint,
            Eigen::ComputeFullV);
        null_space = svd.matrixV().rightCols<4>();
        success = true;
    }
    return null_space;
}

// Multiply two degree one polynomials of variables x, y, z.
// E.g. p1 = a[0]x + a[1]y + a[2]z + a[3]
Eigen::Matrix<double, 1, 20> deg_one_poly_product(const VecX_t& a, const VecX_t& b) {
    Eigen::Matrix<double, 1, 20> product = VecX_t::Zero(20);

    product(poly_xx) = a(poly_x) * b(poly_x); // x*x'
    product(poly_xy)
        = a(poly_x) * b(poly_y) + a(poly_y) * b(poly_x);              // x*y' + y*x'
    product(poly_xz) = a(poly_x) * b(poly_z) + a(poly_z) * b(poly_x); // x*z' + z*x'
    product(poly_yy) = a(poly_y) * b(poly_y);                         // y * y'
    product(poly_yz) = a(poly_y) * b(poly_z) + a(poly_z) * b(poly_y); // y*z' + z * y'
    product(poly_zz) = a(poly_z) * b(poly_z);                         // z * z'
    product(poly_x) = a(poly_x) * b(poly_1) + a(poly_1) * b(poly_x);  // x * c' + c * x'
    product(poly_y) = a(poly_y) * b(poly_1) + a(poly_1) * b(poly_y);  // y * c' + c * y'
    product(poly_z) = a(poly_z) * b(poly_1) + a(poly_1) * b(poly_z);  // z * c' + c * z'
    product(poly_1) = a(poly_1) * b(poly_1);                          // c * c'

    return product;
}

// Multiply a 2 deg poly (in x, y, z) and a one deg poly
Eigen::Matrix<double, 1, 20> deg_two_poly_product(const VecX_t& a, const VecX_t& b) {
    Eigen::Matrix<double, 1, 20> product(20);

    product(poly_xxx) = a(poly_xx) * b(poly_x);
    product(poly_xxy) = a(poly_xx) * b(poly_y)
                        + a(poly_xy) * b(poly_x);
    product(poly_xxz) = a(poly_xx) * b(poly_z)
                        + a(poly_xz) * b(poly_x);
    product(poly_xyy) = a(poly_xy) * b(poly_y)
                        + a(poly_yy) * b(poly_x);
    product(poly_xyz) = a(poly_xy) * b(poly_z)
                        + a(poly_yz) * b(poly_x)
                        + a(poly_xz) * b(poly_y);
    product(poly_xzz) = a(poly_xz) * b(poly_z)
                        + a(poly_zz) * b(poly_x);
    product(poly_yyy) = a(poly_yy) * b(poly_y);
    product(poly_yyz) = a(poly_yy) * b(poly_z)
                        + a(poly_yz) * b(poly_y);
    product(poly_yzz) = a(poly_yz) * b(poly_z)
                        + a(poly_zz) * b(poly_y);
    product(poly_zzz) = a(poly_zz) * b(poly_z);
    product(poly_xx) = a(poly_xx) * b(poly_1)
                       + a(poly_x) * b(poly_x);
    product(poly_xy) = a(poly_xy) * b(poly_1)
                       + a(poly_x) * b(poly_y)
                       + a(poly_y) * b(poly_x);
    product(poly_xz) = a(poly_xz) * b(poly_1)
                       + a(poly_x) * b(poly_z)
                       + a(poly_z) * b(poly_x);
    product(poly_yy) = a(poly_yy) * b(poly_1)
                       + a(poly_y) * b(poly_y);
    product(poly_yz) = a(poly_yz) * b(poly_1)
                       + a(poly_y) * b(poly_z)
                       + a(poly_z) * b(poly_y);
    product(poly_zz) = a(poly_zz) * b(poly_1)
                       + a(poly_z) * b(poly_z);
    product(poly_x) = a(poly_x) * b(poly_1)
                      + a(poly_1) * b(poly_x);
    product(poly_y) = a(poly_y) * b(poly_1)
                      + a(poly_1) * b(poly_y);
    product(poly_z) = a(poly_z) * b(poly_1)
                      + a(poly_1) * b(poly_z);
    product(poly_1) = a(poly_1) * b(poly_1);

    return product;
}

Eigen::Matrix<double, 10, 20> form_polynomial_constraint_matrix(const MatX_t& E_basis) {
    // Build the polynomial form of E
    VecX_t E[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            E[i][j] = VecX_t::Zero(20);
            E[i][j](poly_x) = E_basis(3 * i + j, 0);
            E[i][j](poly_y) = E_basis(3 * i + j, 1);
            E[i][j](poly_z) = E_basis(3 * i + j, 2);
            E[i][j](poly_1) = E_basis(3 * i + j, 3);
        }
    }

    // The constraint matrix we want to construct here.
    Eigen::Matrix<double, 10, 20> M;
    int mrow = 0;

    // Theorem 1: Determinant constraint det(E) = 0 is the first part of M
    M.row(mrow++) = (
        deg_two_poly_product(deg_one_poly_product(E[0][1], E[1][2]) - deg_one_poly_product(E[0][2], E[1][1]), E[2][0]) + 
        deg_two_poly_product(deg_one_poly_product(E[0][2], E[1][0]) - deg_one_poly_product(E[0][0], E[1][2]), E[2][1]) + 
        deg_two_poly_product(deg_one_poly_product(E[0][0], E[1][1]) - deg_one_poly_product(E[0][1], E[1][0]), E[2][2])
    );

    // Theorem 2: the trace constraint: EEtE - 1/2 trace(EEt)E = 0

    // EEt
    Eigen::Matrix<double, 1, 20> EET[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i <= j) {
                EET[i][j] = deg_one_poly_product(E[i][0], E[j][0])
                            + deg_one_poly_product(E[i][1], E[j][1])
                            + deg_one_poly_product(E[i][2], E[j][2]);
            }
            else {
                // EET is symmetric
                EET[i][j] = EET[j][i];
            }
        }
    }

    // EEt - 1/2 trace(EEt)
    Eigen::Matrix<double, 1, 20>(&trace_constraint)[3][3] = EET;
    const Eigen::Matrix<double, 1, 20> trace = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
    for (const int i : {0, 1, 2}) {
        trace_constraint[i][i] -= trace;
    }

    // (EEt - 1/2 trace(EEt)) * E --> EEtE - 1/2 trace(EEt)E = 0
    for (const int i : {0, 1, 2}) {
        for (const int j : {0, 1, 2}) {
            M.row(mrow++) = deg_two_poly_product(trace_constraint[i][0], E[0][j])
                            + deg_two_poly_product(trace_constraint[i][1], E[1][j])
                            + deg_two_poly_product(trace_constraint[i][2], E[2][j]);
        }
    }

    return M;
}

} // namespace stella_vslam

#endif // STELLA_VSLAM_SOLVE_ESSENTIAL_5PT_H
