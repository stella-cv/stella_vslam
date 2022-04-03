#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/marker.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/optimize/global_bundle_adjuster.h"
#include "stella_vslam/optimize/internal/landmark_vertex_container.h"
#include "stella_vslam/optimize/internal/marker_vertex_container.h"
#include "stella_vslam/optimize/internal/se3/shot_vertex_container.h"
#include "stella_vslam/optimize/internal/se3/reproj_edge_wrapper.h"
#include "stella_vslam/optimize/internal/distance_edge.h"
#include "stella_vslam/util/converter.h"

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace stella_vslam {
namespace optimize {

void optimize_impl(g2o::SparseOptimizer& optimizer,
                   std::vector<std::shared_ptr<data::keyframe>>& keyfrms,
                   std::vector<std::shared_ptr<data::landmark>>& lms,
                   std::vector<std::shared_ptr<data::marker>>& markers,
                   std::vector<bool>& is_optimized_lm,
                   internal::se3::shot_vertex_container& keyfrm_vtx_container,
                   internal::landmark_vertex_container& lm_vtx_container,
                   internal::marker_vertex_container& marker_vtx_container,
                   unsigned int num_iter,
                   bool use_huber_kernel,
                   bool* const force_stop_flag) {
    // 2. Construct an optimizer

    std::unique_ptr<g2o::BlockSolverBase> block_solver;
    if (markers.size()) {
        auto linear_solver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
    }
    else {
        auto linear_solver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver));
    }
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    optimizer.setAlgorithm(algorithm);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    // 3. Convert each of the keyframe to the g2o vertex, then set it to the optimizer

    // Set the keyframes to the optimizer
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }

        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(keyfrm, keyfrm->id_ == 0);
        optimizer.addVertex(keyfrm_vtx);
    }

    // 4. Connect the vertices of the keyframe and the landmark by using reprojection edge

    // Container of the reprojection edges
    using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(10 * lms.size());

    // Chi-squared value with significance level of 5%
    // Two degree-of-freedom (n=2)
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // Three degree-of-freedom (n=3)
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    for (unsigned int i = 0; i < lms.size(); ++i) {
        const auto& lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // Convert the landmark to the g2o vertex, then set it to the optimizer
        auto lm_vtx = lm_vtx_container.create_vertex(lm, false);
        optimizer.addVertex(lm_vtx);

        unsigned int num_edges = 0;
        const auto observations = lm->get_observations();
        for (const auto& obs : observations) {
            auto keyfrm = obs.first.lock();
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }

            if (!keyfrm_vtx_container.contain(keyfrm)) {
                continue;
            }

            const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
            const auto& undist_keypt = keyfrm->frm_obs_.undist_keypts_.at(idx);
            const float x_right = keyfrm->frm_obs_.stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->orb_params_->inv_level_sigma_sq_.at(undist_keypt.octave);
            const auto sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                         ? sqrt_chi_sq_2D
                                         : sqrt_chi_sq_3D;
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, lm, lm_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq, use_huber_kernel);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
            ++num_edges;
        }

        if (num_edges == 0) {
            optimizer.removeVertex(lm_vtx);
            is_optimized_lm.at(i) = false;
        }
    }

    // Connect marker vertices
    for (unsigned int marker_idx = 0; marker_idx < markers.size(); ++marker_idx) {
        auto mkr = markers.at(marker_idx);
        if (!mkr) {
            continue;
        }

        // Convert the corners to the g2o vertex, then set it to the optimizer
        auto corner_vertices = marker_vtx_container.create_vertices(mkr, false);
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            const auto corner_vtx = corner_vertices[corner_idx];
            optimizer.addVertex(corner_vtx);

            for (const auto& keyfrm : mkr->observations_) {
                if (!keyfrm) {
                    continue;
                }
                if (keyfrm->will_be_erased()) {
                    continue;
                }
                if (!keyfrm_vtx_container.contain(keyfrm)) {
                    continue;
                }
                const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
                const auto& mkr_2d = keyfrm->markers_2d_.at(mkr->id_);
                const auto& undist_pt = mkr_2d.undist_corners_.at(corner_idx);
                const float x_right = -1.0;
                const float inv_sigma_sq = 1.0;
                auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, nullptr, corner_vtx,
                                                            0, undist_pt.x, undist_pt.y, x_right,
                                                            inv_sigma_sq, 0.0, false);
                reproj_edge_wraps.push_back(reproj_edge_wrap);
                optimizer.addEdge(reproj_edge_wrap.edge_);
            }
        }

        // Add edges for marker shape
        double informationRatio = 1.0e9;
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            const auto dist_edge = new internal::distance_edge();
            dist_edge->setMeasurement(mkr->marker_model_->width_);
            dist_edge->setInformation(MatRC_t<1, 1>::Identity() * informationRatio);
            dist_edge->setVertex(0, corner_vertices[corner_idx]);
            dist_edge->setVertex(1, corner_vertices[(corner_idx + 1) % corner_vertices.size()]);
            optimizer.addEdge(dist_edge);
        }
        for (unsigned int corner_idx = 0; corner_idx < 2; ++corner_idx) {
            const auto dist_edge = new internal::distance_edge();
            const double diagonal_length = std::hypot(mkr->marker_model_->width_, mkr->marker_model_->width_);
            dist_edge->setMeasurement(diagonal_length);
            dist_edge->setInformation(MatRC_t<1, 1>::Identity() * informationRatio);
            dist_edge->setVertex(0, corner_vertices[corner_idx]);
            dist_edge->setVertex(1, corner_vertices[(corner_idx + 2) % corner_vertices.size()]);
            optimizer.addEdge(dist_edge);
        }
    }

    // 5. Perform optimization

    optimizer.initializeOptimization();
    optimizer.optimize(num_iter);

    if (force_stop_flag && *force_stop_flag) {
        return;
    }
}

global_bundle_adjuster::global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter, const bool use_huber_kernel)
    : map_db_(map_db), num_iter_(num_iter), use_huber_kernel_(use_huber_kernel) {}

void global_bundle_adjuster::optimize_for_initialization(bool* const force_stop_flag) const {
    // 1. Collect the dataset
    auto keyfrms = map_db_->get_all_keyframes();
    auto lms = map_db_->get_all_landmarks();
    auto markers = map_db_->get_all_markers();
    std::vector<bool> is_optimized_lm(lms.size(), true);

    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    // Container of the shot vertices
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, keyfrms.size());
    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, lms.size());
    // Container of the landmark vertices
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, markers.size());

    g2o::SparseOptimizer optimizer;

    optimize_impl(optimizer, keyfrms, lms, markers, is_optimized_lm, keyfrm_vtx_container, lm_vtx_container, marker_vtx_container,
                  num_iter_, use_huber_kernel_, force_stop_flag);

    // 6. Extract the result

    for (auto keyfrm : keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
        const auto cam_pose_cw = util::converter::to_eigen_mat(keyfrm_vtx->estimate());

        keyfrm->set_cam_pose(cam_pose_cw);
    }

    for (unsigned int i = 0; i < lms.size(); ++i) {
        if (!is_optimized_lm.at(i)) {
            continue;
        }

        const auto& lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        auto lm_vtx = lm_vtx_container.get_vertex(lm);
        const Vec3_t pos_w = lm_vtx->estimate();

        lm->set_pos_in_world(pos_w);
        lm->update_mean_normal_and_obs_scale_variance();
    }
}

void global_bundle_adjuster::optimize(std::unordered_set<unsigned int>& optimized_keyfrm_ids,
                                      std::unordered_set<unsigned int>& optimized_landmark_ids,
                                      eigen_alloc_unord_map<unsigned int, Vec3_t>& lm_to_pos_w_after_global_BA,
                                      eigen_alloc_unord_map<unsigned int, Mat44_t>& keyfrm_to_pose_cw_after_global_BA,
                                      bool* const force_stop_flag) const {
    // 1. Collect the dataset
    auto keyfrms = map_db_->get_all_keyframes();
    auto lms = map_db_->get_all_landmarks();
    auto markers = map_db_->get_all_markers();
    std::vector<bool> is_optimized_lm(lms.size(), true);

    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    // Container of the shot vertices
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, keyfrms.size());
    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, lms.size());
    // Container of the landmark vertices
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, markers.size());

    g2o::SparseOptimizer optimizer;

    optimize_impl(optimizer, keyfrms, lms, markers, is_optimized_lm, keyfrm_vtx_container, lm_vtx_container, marker_vtx_container,
                  num_iter_, use_huber_kernel_, force_stop_flag);

    // 6. Extract the result

    for (auto keyfrm : keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
        const auto cam_pose_cw = util::converter::to_eigen_mat(keyfrm_vtx->estimate());

        keyfrm_to_pose_cw_after_global_BA[keyfrm->id_] = cam_pose_cw;
        optimized_keyfrm_ids.insert(keyfrm->id_);
    }

    for (unsigned int i = 0; i < lms.size(); ++i) {
        if (!is_optimized_lm.at(i)) {
            continue;
        }

        const auto& lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        auto lm_vtx = lm_vtx_container.get_vertex(lm);
        const Vec3_t pos_w = lm_vtx->estimate();

        lm_to_pos_w_after_global_BA[lm->id_] = pos_w;
        optimized_landmark_ids.insert(lm->id_);
    }

    for (unsigned int marker_idx = 0; marker_idx < markers.size(); ++marker_idx) {
        auto mkr = markers.at(marker_idx);
        if (!mkr) {
            continue;
        }

        eigen_alloc_vector<Vec3_t> corner_pos_w;
        for (unsigned int corner_idx = 0; corner_idx < 4; ++corner_idx) {
            auto corner_vtx = marker_vtx_container.get_vertex(mkr, corner_idx);
            corner_pos_w.push_back(corner_vtx->estimate());
        }
        mkr->set_corner_pos(corner_pos_w);
    }
}

} // namespace optimize
} // namespace stella_vslam
