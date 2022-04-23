#include "stella_vslam/mapping_module.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/module/loop_bundle_adjuster.h"
#include "stella_vslam/optimize/global_bundle_adjuster.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace module {

loop_bundle_adjuster::loop_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter)
    : map_db_(map_db), num_iter_(num_iter) {}

void loop_bundle_adjuster::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void loop_bundle_adjuster::count_loop_BA_execution() {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    ++num_exec_loop_BA_;
}

void loop_bundle_adjuster::abort() {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    abort_loop_BA_ = true;
}

bool loop_bundle_adjuster::is_running() const {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    return loop_BA_is_running_;
}

void loop_bundle_adjuster::optimize() {
    spdlog::info("start loop bundle adjustment");

    unsigned int num_exec_loop_BA = 0;
    {
        std::lock_guard<std::mutex> lock(mtx_thread_);
        loop_BA_is_running_ = true;
        abort_loop_BA_ = false;
        num_exec_loop_BA = num_exec_loop_BA_;
    }

    std::unordered_set<unsigned int> optimized_keyfrm_ids;
    std::unordered_set<unsigned int> optimized_landmark_ids;
    eigen_alloc_unord_map<unsigned int, Vec3_t> lm_to_pos_w_after_global_BA;
    eigen_alloc_unord_map<unsigned int, Mat44_t> keyfrm_to_pose_cw_after_global_BA;
    const auto global_BA = optimize::global_bundle_adjuster(map_db_, num_iter_, false);
    global_BA.optimize(optimized_keyfrm_ids, optimized_landmark_ids,
                       lm_to_pos_w_after_global_BA,
                       keyfrm_to_pose_cw_after_global_BA, &abort_loop_BA_);

    {
        std::lock_guard<std::mutex> lock1(mtx_thread_);

        // if count_loop_BA_execution() was called during the loop BA or the loop BA was aborted,
        // cannot update the map
        if (num_exec_loop_BA != num_exec_loop_BA_ || abort_loop_BA_) {
            spdlog::info("abort loop bundle adjustment");
            loop_BA_is_running_ = false;
            abort_loop_BA_ = false;
            return;
        }

        spdlog::info("finish loop bundle adjustment");
        spdlog::info("updating the map with pose propagation");

        // stop mapping module
        auto future_pause = mapper_->async_pause();
        while (future_pause.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
            while (mapper_->is_terminated()) {
                break;
            }
        }

        std::lock_guard<std::mutex> lock2(data::map_database::mtx_database_);

        eigen_alloc_unord_map<unsigned int, Mat44_t> keyfrm_to_cam_pose_cw_before_BA;
        // update the camera pose along the spanning tree from the origin
        std::list<std::shared_ptr<data::keyframe>> keyfrms_to_check;
        keyfrms_to_check.push_back(map_db_->origin_keyfrm_);
        while (!keyfrms_to_check.empty()) {
            auto parent = keyfrms_to_check.front();
            const Mat44_t cam_pose_wp = parent->get_pose_wc();

            const auto children = parent->graph_node_->get_spanning_children();
            for (auto child : children) {
                if (!optimized_keyfrm_ids.count(child->id_)) {
                    // if `child` is NOT optimized by the loop BA
                    // propagate the pose correction from the spanning parent

                    // parent->child
                    const Mat44_t cam_pose_cp = child->get_pose_cw() * cam_pose_wp;
                    // world->child AFTER correction = parent->child * world->parent AFTER correction
                    keyfrm_to_pose_cw_after_global_BA[child->id_] = cam_pose_cp * keyfrm_to_pose_cw_after_global_BA.at(parent->id_);
                    // check as `child` has been corrected
                    optimized_keyfrm_ids.insert(child->id_);
                }

                // need updating
                keyfrms_to_check.push_back(child);
            }

            // temporally store the camera pose BEFORE correction (for correction of landmark positions)
            keyfrm_to_cam_pose_cw_before_BA[parent->id_] = parent->get_pose_cw();
            // update the camera pose
            parent->set_pose_cw(keyfrm_to_pose_cw_after_global_BA.at(parent->id_));
            // finish updating
            keyfrms_to_check.pop_front();
        }

        // update the positions of the landmarks
        const auto landmarks = map_db_->get_all_landmarks();
        for (const auto& lm : landmarks) {
            if (lm->will_be_erased()) {
                continue;
            }

            if (optimized_landmark_ids.count(lm->id_)) {
                // if `lm` is optimized by the loop BA

                // update with the optimized position
                lm->set_pos_in_world(lm_to_pos_w_after_global_BA.at(lm->id_));
            }
            else {
                // if `lm` is NOT optimized by the loop BA

                // correct the position according to the move of the camera pose of the reference keyframe
                auto ref_keyfrm = lm->get_ref_keyframe();

                assert(optimized_keyfrm_ids.count(ref_keyfrm->id_));

                // convert the position to the camera-reference using the camera pose BEFORE the correction
                const Mat44_t pose_cw_before_BA = keyfrm_to_cam_pose_cw_before_BA.at(ref_keyfrm->id_);
                const Mat33_t rot_cw_before_BA = pose_cw_before_BA.block<3, 3>(0, 0);
                const Vec3_t trans_cw_before_BA = pose_cw_before_BA.block<3, 1>(0, 3);
                const Vec3_t pos_c = rot_cw_before_BA * lm->get_pos_in_world() + trans_cw_before_BA;

                // convert the position to the world-reference using the camera pose AFTER the correction
                const Mat44_t cam_pose_wc = ref_keyfrm->get_pose_wc();
                const Mat33_t rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                const Vec3_t trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                lm->set_pos_in_world(rot_wc * pos_c + trans_wc);
            }
        }

        mapper_->resume();
        loop_BA_is_running_ = false;

        spdlog::info("updated the map");
    }
}

} // namespace module
} // namespace stella_vslam
