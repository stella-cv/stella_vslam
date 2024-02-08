#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>

#include "stella_vslam/type.h"

namespace solver_5pt {

// In the following code, polynomials are expressed as vectors containing
// their coeficients in the basis of monomials:
//
//  [xxx xxy xyy yyy xxz xyz yyz xzz yzz zzz xx xy yy xz yz zz x y z 1]
//
// Note that there is an error in Stewenius' paper.  In equation (9) they
// propose to use the basis:
//
//  [xxx xxy xxz xyy xyz xzz yyy yyz yzz zzz xx xy xz yy yz zz x y z 1]
//
// But this is not the basis used in the rest of the paper, neither in
// the code they provide. I (pau) have spend 4 hours debugging and
// reverse engineering their code to find the problem. :(
enum
{
  coef_xxx,
  coef_xxy,
  coef_xyy,
  coef_yyy,
  coef_xxz,
  coef_xyz,
  coef_yyz,
  coef_xzz,
  coef_yzz,
  coef_zzz,
  coef_xx,
  coef_xy,
  coef_yy,
  coef_xz,
  coef_yz,
  coef_zz,
  coef_x,
  coef_y,
  coef_z,
  coef_1
};

/// 4d vector using double internal format
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;

/// 9d vector using double internal format
using Vec9 = Eigen::Matrix<double, 9, 1>;

/// 3x3 matrix using double internal format
using Mat3 = Eigen::Matrix<double, 3, 3>;

//-- General purpose Matrix and Vector
/// Unconstrained matrix using double internal format
using Mat = Eigen::MatrixXd;

/// Unconstrained vector using double internal format
using Vec = Eigen::VectorXd;

/// 3xN matrix using double internal format
using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

inline void EncodeEpipolarEquation(const stella_vslam::eigen_alloc_vector<Vec3>& x1, const stella_vslam::eigen_alloc_vector<Vec3>& x2, Mat* A) {
    for (size_t i = 0; i < x1.size(); ++i) {
        A->row(i) << x2.at(i)(0) * x1.at(i).transpose(),
            x2.at(i)(1) * x1.at(i).transpose(),
            x2.at(i)(2) * x1.at(i).transpose();
    }
}

Mat FivePointsNullspaceBasis(const stella_vslam::eigen_alloc_vector<Vec3> &x1, const stella_vslam::eigen_alloc_vector<Vec3> &x2) {
  Mat epipolar_constraint = Eigen::Matrix<double,9, 9>::Constant(0.0);
  EncodeEpipolarEquation(x1, x2, &epipolar_constraint);
  Eigen::SelfAdjointEigenSolver<Mat> solver
    (epipolar_constraint.transpose() * epipolar_constraint);
  return solver.eigenvectors().leftCols<4>();
}

Vec o1(const Vec &a, const Vec &b) {
  Vec res = Vec::Zero(20);

  res(coef_xx) = a(coef_x) * b(coef_x);
  res(coef_xy) = a(coef_x) * b(coef_y)
               + a(coef_y) * b(coef_x);
  res(coef_xz) = a(coef_x) * b(coef_z)
               + a(coef_z) * b(coef_x);
  res(coef_yy) = a(coef_y) * b(coef_y);
  res(coef_yz) = a(coef_y) * b(coef_z)
               + a(coef_z) * b(coef_y);
  res(coef_zz) = a(coef_z) * b(coef_z);
  res(coef_x)  = a(coef_x) * b(coef_1)
               + a(coef_1) * b(coef_x);
  res(coef_y)  = a(coef_y) * b(coef_1)
               + a(coef_1) * b(coef_y);
  res(coef_z)  = a(coef_z) * b(coef_1)
               + a(coef_1) * b(coef_z);
  res(coef_1)  = a(coef_1) * b(coef_1);

  return res;
}

Vec o2(const Vec &a, const Vec &b) {
  Vec res(20);

  res(coef_xxx) = a(coef_xx) * b(coef_x);
  res(coef_xxy) = a(coef_xx) * b(coef_y)
                + a(coef_xy) * b(coef_x);
  res(coef_xxz) = a(coef_xx) * b(coef_z)
                + a(coef_xz) * b(coef_x);
  res(coef_xyy) = a(coef_xy) * b(coef_y)
                + a(coef_yy) * b(coef_x);
  res(coef_xyz) = a(coef_xy) * b(coef_z)
                + a(coef_yz) * b(coef_x)
                + a(coef_xz) * b(coef_y);
  res(coef_xzz) = a(coef_xz) * b(coef_z)
                + a(coef_zz) * b(coef_x);
  res(coef_yyy) = a(coef_yy) * b(coef_y);
  res(coef_yyz) = a(coef_yy) * b(coef_z)
                + a(coef_yz) * b(coef_y);
  res(coef_yzz) = a(coef_yz) * b(coef_z)
                + a(coef_zz) * b(coef_y);
  res(coef_zzz) = a(coef_zz) * b(coef_z);
  res(coef_xx)  = a(coef_xx) * b(coef_1)
                + a(coef_x)  * b(coef_x);
  res(coef_xy)  = a(coef_xy) * b(coef_1)
                + a(coef_x)  * b(coef_y)
                + a(coef_y)  * b(coef_x);
  res(coef_xz)  = a(coef_xz) * b(coef_1)
                + a(coef_x)  * b(coef_z)
                + a(coef_z)  * b(coef_x);
  res(coef_yy)  = a(coef_yy) * b(coef_1)
                + a(coef_y)  * b(coef_y);
  res(coef_yz)  = a(coef_yz) * b(coef_1)
                + a(coef_y)  * b(coef_z)
                + a(coef_z)  * b(coef_y);
  res(coef_zz)  = a(coef_zz) * b(coef_1)
                + a(coef_z)  * b(coef_z);
  res(coef_x)   = a(coef_x)  * b(coef_1)
                + a(coef_1)  * b(coef_x);
  res(coef_y)   = a(coef_y)  * b(coef_1)
                + a(coef_1)  * b(coef_y);
  res(coef_z)   = a(coef_z)  * b(coef_1)
                + a(coef_1)  * b(coef_z);
  res(coef_1)   = a(coef_1)  * b(coef_1);

  return res;
}

Mat FivePointsPolynomialConstraints(const Mat &E_basis) {
  // Build the polynomial form of E (equation (8) in Stewenius et al. [1])
  Vec E[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      E[i][j] = Vec::Zero(20);
      E[i][j](coef_x) = E_basis(3 * i + j, 0);
      E[i][j](coef_y) = E_basis(3 * i + j, 1);
      E[i][j](coef_z) = E_basis(3 * i + j, 2);
      E[i][j](coef_1) = E_basis(3 * i + j, 3);
    }
  }

  // The constraint matrix.
  Mat M(10, 20);
  int mrow = 0;

  // Determinant constraint det(E) = 0; equation (19) of Nister [2].
  M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0]) +
                  o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1]) +
                  o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

  // Cubic singular values constraint.
  // Equation (20).
  Vec EET[3][3];
  for (int i = 0; i < 3; ++i) {    // Since EET is symmetric, we only compute
    for (int j = 0; j < 3; ++j) {  // its upper triangular part.
      if (i <= j) {
        EET[i][j] = o1(E[i][0], E[j][0])
                  + o1(E[i][1], E[j][1])
                  + o1(E[i][2], E[j][2]);
      } else {
        EET[i][j] = EET[j][i];
      }
    }
  }

  // Equation (21).
  Vec (&L)[3][3] = EET;
  const Vec trace  = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
  for (const int i : {0,1,2}) {
    L[i][i] -= trace;
  }

  // Equation (23).
  for (const int i : {0,1,2}) {
    for (const int j : {0,1,2}) {
      Vec LEij = o2(L[i][0], E[0][j])
               + o2(L[i][1], E[1][j])
               + o2(L[i][2], E[2][j]);
      M.row(mrow++) = LEij;
    }
  }

  return M;
}

void FivePointsRelativePose(const stella_vslam::eigen_alloc_vector<Vec3> &x1,
                            const stella_vslam::eigen_alloc_vector<Vec3> &x2,
                            std::vector<Mat3> *Es) {
  // Step 1: Nullspace Extraction.
  const Eigen::Matrix<double, 9, 4> E_basis = FivePointsNullspaceBasis(x1, x2);

  // Step 2: Constraint Expansion.
  const Eigen::Matrix<double, 10, 20> E_constraints = FivePointsPolynomialConstraints(E_basis);

  // Step 3: Gauss-Jordan Elimination (done thanks to a LU decomposition).
  using Mat10 = Eigen::Matrix<double, 10, 10>;
  Eigen::FullPivLU<Mat10> c_lu(E_constraints.block<10, 10>(0, 0));
  const Mat10 M = c_lu.solve(E_constraints.block<10, 10>(0, 10));

  // For next steps we follow the matlab code given in Stewenius et al [1].

  // Build action matrix.

  const Mat10 & B = M.topRightCorner<10,10>();
  Mat10 At = Mat10::Zero(10,10);
  At.block<3, 10>(0, 0) = B.block<3, 10>(0, 0);
  At.row(3) = B.row(4);
  At.row(4) = B.row(5);
  At.row(5) = B.row(7);
  At(6,0) = At(7,1) = At(8,3) = At(9,6) = -1;

  Eigen::EigenSolver<Mat10> eigensolver(At);
  const auto& eigenvectors = eigensolver.eigenvectors();
  const auto& eigenvalues = eigensolver.eigenvalues();

  // Build essential matrices for the real solutions.
  Es->reserve(10);
  for (int s = 0; s < 10; ++s) {
    // Only consider real solutions.
    if (eigenvalues(s).imag() != 0) {
      continue;
    }
    Mat3 E;
    Eigen::Map<Vec9 >(E.data()) =
        E_basis * eigenvectors.col(s).tail<4>().real();
    Es->emplace_back(E.transpose());
  }
}
} // namespace minimal_solver