#include "stella_vslam/type.h"
#include "stella_vslam/mapping_module.h"
#include "stella_vslam/global_optimization_module.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/match/fuse.h"
#include "stella_vslam/match/robust.h"
#include "stella_vslam/module/two_view_triangulator.h"
#include "stella_vslam/solve/essential_solver.h"

#include <unordered_set>
#include <thread>

#include <spdlog/spdlog.h>

namespace stella_vslam {

mapping_module::mapping_module(const YAML::Node& yaml_node, data::map_database* map_db, data::bow_database* bow_db, data::bow_vocabulary* bow_vocab)
    : local_map_cleaner_(new module::local_map_cleaner(yaml_node, map_db, bow_db)),
      map_db_(map_db), bow_db_(bow_db), bow_vocab_(bow_vocab),
      local_bundle_adjuster_(new optimize::local_bundle_adjuster(yaml_node)) {
    spdlog::debug("CONSTRUCT: mapping_module");
    spdlog::debug("load mapping parameters");

    spdlog::debug("load monocular mappping parameters");
    if (yaml_node["baseline_dist_thr"]) {
        if (yaml_node["baseline_dist_thr_ratio"]) {
            throw std::runtime_error("Do not set both baseline_dist_thr_ratio and baseline_dist_thr.");
        }
        baseline_dist_thr_ = yaml_node["baseline_dist_thr"].as<double>(1.0);
        use_baseline_dist_thr_ratio_ = false;
        spdlog::debug("Use baseline_dist_thr: {}", baseline_dist_thr_);
    }
    else {
        baseline_dist_thr_ratio_ = yaml_node["baseline_dist_thr_ratio"].as<double>(0.02);
        use_baseline_dist_thr_ratio_ = true;
        spdlog::debug("Use baseline_dist_thr_ratio: {}", baseline_dist_thr_ratio_);
    }
}

mapping_module::~mapping_module() {
    spdlog::debug("DESTRUCT: mapping_module");
}

void mapping_module::set_tracking_module(tracking_module* tracker) {
    tracker_ = tracker;
}

void mapping_module::set_global_optimization_module(global_optimization_module* global_optimizer) {
    global_optimizer_ = global_optimizer;
}

void mapping_module::run() {
    spdlog::info("start mapping module");

    is_terminated_ = false;
    set_is_idle(true);

    while (true) {
        // waiting time for the other threads
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // check if termination is requested
        if (terminate_is_requested()) {
            // terminate and break
            terminate();
            break;
        }

        // check if pause is requested and not prevented
        if (pause_is_requested_and_not_prevented()) {
            set_is_idle(false);
            // if any keyframe is queued, all of them must be processed before the pause
            while (keyframe_is_queued()) {
                // create and extend the map with the new keyframe
                mapping_with_new_keyframe();
                // send the new keyframe to the global optimization module
                global_optimizer_->queue_keyframe(cur_keyfrm_);
            }
            set_is_idle(true);
            // pause and wait
            pause();
            // check if termination or reset is requested during pause
            while (is_paused() && !terminate_is_requested() && !reset_is_requested()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }

        // check if reset is requested
        if (reset_is_requested()) {
            // reset and continue
            reset();
            set_is_idle(true);
            continue;
        }

        // if the queue is empty, the following process is not needed
        if (!keyframe_is_queued()) {
            set_is_idle(true);
            continue;
        }

        set_is_idle(false);
        // create and extend the map with the new keyframe
        mapping_with_new_keyframe();
        // send the new keyframe to the global optimization module
        global_optimizer_->queue_keyframe(cur_keyfrm_);
    }

    spdlog::info("terminate mapping module");
}

void mapping_module::queue_keyframe(const std::shared_ptr<data::keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    keyfrms_queue_.push_back(keyfrm);
    abort_local_BA_ = true;
}

unsigned int mapping_module::get_num_queued_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return keyfrms_queue_.size();
}

bool mapping_module::keyframe_is_queued() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return !keyfrms_queue_.empty();
}

bool mapping_module::is_idle() const {
    return is_idle_;
}

void mapping_module::set_is_idle(const bool is_idle) {
    is_idle_ = is_idle;
}

bool mapping_module::is_skipping_localBA() const {
    auto queued_keyframes = get_num_queued_keyframes();
    return queued_keyframes >= queue_threshold_;
}

void mapping_module::abort_local_BA() {
    abort_local_BA_ = true;
}

void mapping_module::mapping_with_new_keyframe() {
    // dequeue
    {
        std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
        // dequeue -> cur_keyfrm_
        cur_keyfrm_ = keyfrms_queue_.front();
        keyfrms_queue_.pop_front();
    }

    // set the origin keyframe
    local_map_cleaner_->set_origin_keyframe_id(map_db_->origin_keyfrm_->id_);

    // store the new keyframe to the database
    store_new_keyframe();

    // remove redundant landmarks
    local_map_cleaner_->remove_redundant_landmarks(cur_keyfrm_->id_);

    // triangulate new landmarks between the current frame and each of the covisibilities
    create_new_landmarks();

    if (keyframe_is_queued()) {
        return;
    }

    // detect and resolve the duplication of the landmarks observed in the current frame
    update_new_keyframe();

    if (keyframe_is_queued() || pause_is_requested()) {
        return;
    }

    // local bundle adjustment
    abort_local_BA_ = false;
    // If the processing speed is insufficient, skip localBA.
    if (2 < map_db_->get_num_keyframes()) {
        if (is_skipping_localBA()) {
            spdlog::debug("Skipped localBA due to insufficient performance");
        }
        else {
            local_bundle_adjuster_->optimize(map_db_, cur_keyfrm_, &abort_local_BA_);
        }
    }
    local_map_cleaner_->remove_redundant_keyframes(cur_keyfrm_);
}

void mapping_module::store_new_keyframe() {
    // compute BoW feature vector
    if (!cur_keyfrm_->bow_is_available()) {
        cur_keyfrm_->compute_bow(bow_vocab_);
    }

    // update graph
    const auto cur_lms = cur_keyfrm_->get_landmarks();
    for (unsigned int idx = 0; idx < cur_lms.size(); ++idx) {
        auto lm = cur_lms.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // if `lm` does not have the observation information from `cur_keyfrm_`,
        // add the association between the keyframe and the landmark
        if (lm->is_observed_in_keyframe(cur_keyfrm_)) {
            // if `lm` is correctly observed, make it be checked by the local map cleaner
            local_map_cleaner_->add_fresh_landmark(lm);
            continue;
        }

        // update connection
        lm->add_observation(cur_keyfrm_, idx);
        // update geometry
        lm->update_mean_normal_and_obs_scale_variance();
        lm->compute_descriptor();
    }
    cur_keyfrm_->graph_node_->update_connections();

    // store the new keyframe to the map database
    map_db_->add_keyframe(cur_keyfrm_);
}

void mapping_module::create_new_landmarks() {
    // get the covisibilities of `cur_keyfrm_`
    // in order to triangulate landmarks between `cur_keyfrm_` and each of the covisibilities
    constexpr unsigned int num_covisibilities = 10;
    const unsigned int heuristic_ratio = cur_keyfrm_->depth_is_available() ? 1 : 2;
    const auto cur_covisibilities = cur_keyfrm_->graph_node_->get_top_n_covisibilities(num_covisibilities * heuristic_ratio);

    // lowe's_ratio will not be used
    match::robust robust_matcher(0.0, false);

    // camera center of the current keyframe
    const Vec3_t cur_cam_center = cur_keyfrm_->get_trans_wc();

    for (unsigned int i = 0; i < cur_covisibilities.size(); ++i) {
        // if any keyframe is queued, abort the triangulation
        if (1 < i && keyframe_is_queued()) {
            return;
        }

        // get the neighbor keyframe
        auto ngh_keyfrm = cur_covisibilities.at(i);

        // camera center of the neighbor keyframe
        const Vec3_t ngh_cam_center = ngh_keyfrm->get_trans_wc();

        // compute the baseline between the current and neighbor keyframes
        const Vec3_t baseline_vec = ngh_cam_center - cur_cam_center;
        const auto baseline_dist = baseline_vec.norm();

        // if the scene scale is much smaller than the baseline, abort the triangulation
        if (use_baseline_dist_thr_ratio_) {
            const float median_depth_in_ngh = ngh_keyfrm->compute_median_depth(true);
            if (baseline_dist < baseline_dist_thr_ratio_ * median_depth_in_ngh) {
                continue;
            }
        }
        else {
            if (baseline_dist < baseline_dist_thr_) {
                continue;
            }
        }

        // estimate matches between the current and neighbor keyframes,
        // then reject outliers using Essential matrix computed from the two camera poses

        // (cur bearing) * E_ngh_to_cur * (ngh bearing) = 0
        // const Mat33_t E_ngh_to_cur = solve::essential_solver::create_E_21(ngh_keyfrm, cur_keyfrm_);
        const Mat33_t E_ngh_to_cur = solve::essential_solver::create_E_21(ngh_keyfrm->get_rot_cw(), ngh_keyfrm->get_trans_cw(),
                                                                          cur_keyfrm_->get_rot_cw(), cur_keyfrm_->get_trans_cw());

        // vector of matches (idx in the current, idx in the neighbor)
        std::vector<std::pair<unsigned int, unsigned int>> matches;
        robust_matcher.match_for_triangulation(cur_keyfrm_, ngh_keyfrm, E_ngh_to_cur, matches);

        // triangulation
        triangulate_with_two_keyframes(cur_keyfrm_, ngh_keyfrm, matches);
    }
}

void mapping_module::triangulate_with_two_keyframes(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2,
                                                    const std::vector<std::pair<unsigned int, unsigned int>>& matches) {
    const module::two_view_triangulator triangulator(keyfrm_1, keyfrm_2, 1.0);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (unsigned int i = 0; i < matches.size(); ++i) {
        const auto idx_1 = matches.at(i).first;
        const auto idx_2 = matches.at(i).second;

        // triangulate between idx_1 and idx_2
        Vec3_t pos_w;
        if (!triangulator.triangulate(idx_1, idx_2, pos_w)) {
            // failed
            continue;
        }
        // succeeded

        // create a landmark object
        auto lm = std::make_shared<data::landmark>(pos_w, keyfrm_1);

        lm->add_observation(keyfrm_1, idx_1);
        lm->add_observation(keyfrm_2, idx_2);

        keyfrm_1->add_landmark(lm, idx_1);
        keyfrm_2->add_landmark(lm, idx_2);

        lm->compute_descriptor();
        lm->update_mean_normal_and_obs_scale_variance();

        map_db_->add_landmark(lm);
        // wait for redundancy check
#ifdef USE_OPENMP
#pragma omp critical
#endif
        {
            local_map_cleaner_->add_fresh_landmark(lm);
        }
    }
}

void mapping_module::update_new_keyframe() {
    // get the targets to check landmark fusion
    const unsigned int num_covisibilities = cur_keyfrm_->depth_is_available() ? 10 : 20;
    const auto fuse_tgt_keyfrms = get_second_order_covisibilities(num_covisibilities, 5);

    // resolve the duplication of landmarks between the current keyframe and the targets
    fuse_landmark_duplication(fuse_tgt_keyfrms);

    // update the geometries
    const auto cur_landmarks = cur_keyfrm_->get_landmarks();
    for (const auto& lm : cur_landmarks) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        lm->compute_descriptor();
        lm->update_mean_normal_and_obs_scale_variance();
    }

    // update the graph
    cur_keyfrm_->graph_node_->update_connections();
}

std::unordered_set<std::shared_ptr<data::keyframe>> mapping_module::get_second_order_covisibilities(const unsigned int first_order_thr,
                                                                                                    const unsigned int second_order_thr) {
    const auto cur_covisibilities = cur_keyfrm_->graph_node_->get_top_n_covisibilities(first_order_thr);

    std::unordered_set<std::shared_ptr<data::keyframe>> fuse_tgt_keyfrms;
    fuse_tgt_keyfrms.reserve(cur_covisibilities.size() * 2);

    for (const auto& first_order_covis : cur_covisibilities) {
        if (first_order_covis->will_be_erased()) {
            continue;
        }

        // check if the keyframe is aleady inserted
        if (static_cast<bool>(fuse_tgt_keyfrms.count(first_order_covis))) {
            continue;
        }

        fuse_tgt_keyfrms.insert(first_order_covis);

        // get the covisibilities of the covisibility of the current keyframe
        const auto ngh_covisibilities = first_order_covis->graph_node_->get_top_n_covisibilities(second_order_thr);
        for (const auto& second_order_covis : ngh_covisibilities) {
            if (second_order_covis->will_be_erased()) {
                continue;
            }
            // "the covisibilities of the covisibility" contains the current keyframe
            if (*second_order_covis == *cur_keyfrm_) {
                continue;
            }

            fuse_tgt_keyfrms.insert(second_order_covis);
        }
    }

    return fuse_tgt_keyfrms;
}

void mapping_module::fuse_landmark_duplication(const std::unordered_set<std::shared_ptr<data::keyframe>>& fuse_tgt_keyfrms) {
    match::fuse matcher;

    {
        // reproject the landmarks observed in the current keyframe to each of the targets, and acquire
        // - additional matches
        // - duplication of matches
        // then, add matches and solve duplication
        auto cur_landmarks = cur_keyfrm_->get_landmarks();
        for (const auto& fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            matcher.replace_duplication(map_db_, fuse_tgt_keyfrm, cur_landmarks);
        }
    }

    {
        // reproject the landmarks observed in each of the targets to each of the current frame, and acquire
        // - additional matches
        // - duplication of matches
        // then, add matches and solve duplication
        std::unordered_set<std::shared_ptr<data::landmark>> candidate_landmarks_to_fuse;
        candidate_landmarks_to_fuse.reserve(fuse_tgt_keyfrms.size() * cur_keyfrm_->frm_obs_.num_keypts_);

        for (const auto& fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            const auto fuse_tgt_landmarks = fuse_tgt_keyfrm->get_landmarks();

            for (const auto& lm : fuse_tgt_landmarks) {
                if (!lm) {
                    continue;
                }
                if (lm->will_be_erased()) {
                    continue;
                }

                if (static_cast<bool>(candidate_landmarks_to_fuse.count(lm))) {
                    continue;
                }
                candidate_landmarks_to_fuse.insert(lm);
            }
        }

        matcher.replace_duplication(map_db_, cur_keyfrm_, candidate_landmarks_to_fuse);
    }
}

std::future<void> mapping_module::async_reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    reset_is_requested_ = true;
    promises_reset_.emplace_back();
    return promises_reset_.back().get_future();
}

bool mapping_module::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void mapping_module::reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    spdlog::info("reset mapping module");
    keyfrms_queue_.clear();
    local_map_cleaner_->reset();
    reset_is_requested_ = false;
    for (auto& promise : promises_reset_) {
        promise.set_value();
    }
    promises_reset_.clear();
}

std::future<void> mapping_module::async_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
    std::lock_guard<std::mutex> lock2(mtx_keyfrm_queue_);
    abort_local_BA_ = true;
    promises_pause_.emplace_back();
    return promises_pause_.back().get_future();
}

bool mapping_module::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

bool mapping_module::pause_is_requested_and_not_prevented() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_ && !prevent_pause_;
}

bool mapping_module::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

void mapping_module::pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    spdlog::info("pause mapping module");
    is_paused_ = true;
    for (auto& promise : promises_pause_) {
        promise.set_value();
    }
    promises_pause_.clear();
}

bool mapping_module::prevent_pause_if_not_paused() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (!is_paused_) {
        prevent_pause_ = true;
    }
    return prevent_pause_;
}

void mapping_module::stop_prevent_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    prevent_pause_ = false;
}

void mapping_module::resume() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);

    // if it has been already terminated, cannot resume
    if (is_terminated_) {
        return;
    }

    assert(keyfrms_queue_.empty());

    is_paused_ = false;
    pause_is_requested_ = false;

    spdlog::info("resume mapping module");
}

std::future<void> mapping_module::async_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
    promises_terminate_.emplace_back();
    return promises_terminate_.back().get_future();
}

bool mapping_module::is_terminated() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool mapping_module::terminate_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void mapping_module::terminate() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);
    is_paused_ = true;
    is_terminated_ = true;
    set_is_idle(true);
    for (auto& promise : promises_terminate_) {
        promise.set_value();
    }
    promises_terminate_.clear();
}

} // namespace stella_vslam
