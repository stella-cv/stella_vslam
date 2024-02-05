#include "helper/landmark.h"
#include "helper/bearing_vector.h"

#include "stella_vslam/type.h"
#include "stella_vslam/util/converter.h"
#include "stella_vslam/solve/pnp_solver.h"

#include <memory>

#include <opencv2/core/types.hpp>

#include <gtest/gtest.h>

using namespace stella_vslam;

TEST(pnp_solver, compute_pose) {
    eigen_alloc_vector<Vec3_t> points;
    points.emplace_back(Vec3_t{0.90285978902599595131, 59.132259867903883332, -77.283655667882285911});
    points.emplace_back(Vec3_t{69.49294443595013604, 60.215644217552778628, 1.5243671019389921639});
    points.emplace_back(Vec3_t{-61.303331688537312516, -82.61571787600384198, 81.583023085091554094});
    points.emplace_back(Vec3_t{75.897304024121126531, 17.910225253366846232, -3.2652482029668306041});

    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, points, bearings);

    Mat33_t rot_cw;
    Vec3_t trans_cw;
    unsigned int gauss_newton_num_iter = 10; // If less than 6, the test fails.
    solve::pnp_solver::compute_pose(bearings, points, rot_cw, trans_cw, gauss_newton_num_iter);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot_cw.transpose()).norm();
    const auto trans_err = (trans_gt - trans_cw).norm();
    EXPECT_LT(rot_err, 1e-2);
    EXPECT_LT(trans_err, 1);
}

TEST(pnp_solver, without_ransac) {
    // Create four landmarks which needed the least number for solve problem
    const unsigned int num_landmarks = 6;
    eigen_alloc_vector<Vec3_t> points;
    points.emplace_back(Vec3_t{-19.283677, -18.130606, 82.329830});
    points.emplace_back(Vec3_t{-88.230105, 61.669552, -52.896303});
    points.emplace_back(Vec3_t{-46.048140, 47.097662, -92.047191});
    points.emplace_back(Vec3_t{-91.468185, -56.584450, -35.762650});
    points.emplace_back(Vec3_t{-82.214075, 11.124351, -0.022995});
    points.emplace_back(Vec3_t{-88.117582, 84.359816, -55.239983});

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors from pose and landmarks
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, points, bearings);

    // octaves and scale_factor are required of solver
    // In this test, octave is 0 and scale factor is 1 for each keypoint
    std::vector<int> octaves(num_landmarks, 0);
    const std::vector<float> scale_factor{1};

    // Compute the camera pose by pnp_solver
    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, octaves, points, scale_factor, 0, true));
    solver->find_via_ransac(10);

    EXPECT_TRUE(solver->solution_is_valid());

    // Compute error of estimated pose
    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot.transpose()).norm();
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-4);
    EXPECT_LT(trans_err, 1e-4);
}

TEST(pnp_solver, without_outlier) {
    // Create landmarks
    const unsigned int num_landmarks = 100;
    const auto points = create_random_landmarks_in_space(num_landmarks, 100);

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors from pose and landmarks
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, points, bearings);

    // octaves and scale_factor are required of solver
    // In this test, octave is 0 and scale factor is 1 for each keypoint
    std::vector<int> octaves(num_landmarks, 0);
    const std::vector<float> scale_factor{1};

    // Compute the camera pose by pnp_solver
    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, octaves, points, scale_factor, 10, true));
    solver->find_via_ransac(30);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot.transpose()).norm();
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-4);
    EXPECT_LT(trans_err, 1e-4);
}

TEST(pnp_solver, with_outlier) {
    // Create landmarks
    const unsigned int num_landmarks = 100;
    const auto points = create_random_landmarks_in_space(num_landmarks, 100);

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors containing observation noise
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, points, bearings);
    const double outlier_ratio = 0.3;
    add_noise(bearings, 0.1, outlier_ratio);

    // octaves and scale_factor are required of solver
    // In this test, octave is 0 and scale factor is 1 for each keypoint
    std::vector<int> octaves(num_landmarks, 0);
    const std::vector<float> scale_factor{1};

    // Compute the camera pose by pnp_solver
    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, octaves, points, scale_factor, 10, true));
    solver->find_via_ransac(50, false);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot.transpose()).norm();
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-2);
    EXPECT_LT(trans_err, 1);
}

TEST(pnp_solver, with_outlier_and_noise) {
    // Create landmarks
    const unsigned int num_landmarks = 200;
    const auto points = create_random_landmarks_in_space(num_landmarks, 100);

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors containing observation noise
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, points, bearings);
    const double outlier_ratio = 0.3;
    add_noise(bearings, 0.1, outlier_ratio);
    add_noise(bearings, 0.005, 1.0);

    // octaves and scale_factor are required of solver
    // In this test, octave is 0 and scale factor is 1 for each keypoint
    std::vector<int> octaves(num_landmarks, 0);
    const std::vector<float> scale_factor{1};

    // Compute the camera pose by pnp_solver
    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, octaves, points, scale_factor, 10, true));
    solver->find_via_ransac(50, false);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot.transpose()).norm();
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-2);
    EXPECT_LT(trans_err, 1);
}
