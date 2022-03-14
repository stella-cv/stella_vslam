#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/camera_database.h"
#include "openvslam/data/orb_params_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/io/map_database_io.h"
#include "openvslam/data/common.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <fstream>

namespace openvslam {
namespace io {

map_database_io::map_database_io(data::camera_database* cam_db, data::orb_params_database* orb_params_db, data::map_database* map_db,
                                 data::bow_database* bow_db, data::bow_vocabulary* bow_vocab)
    : cam_db_(cam_db), orb_params_db_(orb_params_db), map_db_(map_db), bow_db_(bow_db), bow_vocab_(bow_vocab) {}

void map_database_io::save_message_pack(const std::string& path) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    assert(cam_db_ && orb_params_db_ && map_db_);
    const auto cameras = cam_db_->to_json();
    const auto orb_params = orb_params_db_->to_json();
    nlohmann::json keyfrms;
    nlohmann::json landmarks;
    map_db_->to_json(keyfrms, landmarks);

    nlohmann::json json{{"cameras", cameras},
                        {"orb_params", orb_params},
                        {"keyframes", keyfrms},
                        {"landmarks", landmarks},
                        {"frame_next_id", static_cast<unsigned int>(data::frame::next_id_)},
                        {"keyframe_next_id", static_cast<unsigned int>(data::keyframe::next_id_)},
                        {"landmark_next_id", static_cast<unsigned int>(data::landmark::next_id_)}};

    std::ofstream ofs(path, std::ios::out | std::ios::binary);

    if (ofs.is_open()) {
        spdlog::info("save the MessagePack file of database to {}", path);
        const auto msgpack = nlohmann::json::to_msgpack(json);
        ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
        ofs.close();
    }
    else {
        spdlog::critical("cannot create a file at {}", path);
    }
}

void map_database_io::load_message_pack(const std::string& path) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. initialize database

    assert(cam_db_ && orb_params_db_ && map_db_ && bow_db_ && bow_vocab_);
    map_db_->clear();
    bow_db_->clear();

    // 2. load binary bytes

    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        spdlog::critical("cannot load the file at {}", path);
        throw std::runtime_error("cannot load the file at " + path);
    }

    spdlog::info("load the MessagePack file of database from {}", path);
    std::vector<uint8_t> msgpack;
    while (true) {
        uint8_t buffer;
        ifs.read(reinterpret_cast<char*>(&buffer), sizeof(uint8_t));
        if (ifs.eof()) {
            break;
        }
        msgpack.push_back(buffer);
    }
    ifs.close();

    // 3. parse into JSON

    const auto json = nlohmann::json::from_msgpack(msgpack);

    // 4. load database

    // load static variables
    data::frame::next_id_ = json.at("frame_next_id").get<unsigned int>();
    data::keyframe::next_id_ = json.at("keyframe_next_id").get<unsigned int>();
    data::landmark::next_id_ = json.at("landmark_next_id").get<unsigned int>();
    // load database
    const auto json_cameras = json.at("cameras");
    cam_db_->from_json(json_cameras);
    const auto json_orb_params = json.at("orb_params");
    orb_params_db_->from_json(json_orb_params);
    const auto json_keyfrms = json.at("keyframes");
    const auto json_landmarks = json.at("landmarks");
    map_db_->from_json(cam_db_, orb_params_db_, bow_vocab_, json_keyfrms, json_landmarks);
    const auto keyfrms = map_db_->get_all_keyframes();
    for (const auto& keyfrm : keyfrms) {
        bow_db_->add_keyframe(keyfrm);
    }
}

void map_database_io::load_new_message_pack(const std::string& path, const Mat44_t transf_matrix, float scale_factor) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. Check if exists a map databse already
    assert(cam_db_ && map_db_ && bow_db_ && bow_vocab_);

    // 2. Load binary bytes

    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        spdlog::critical("cannot load the file at {}", path);
        throw std::runtime_error("cannot load the file at " + path);
    }

    spdlog::info("load the MessagePack file of database from {}", path);
    std::vector<uint8_t> msgpack;
    while (true) {
        uint8_t buffer;
        ifs.read(reinterpret_cast<char*>(&buffer), sizeof(uint8_t));
        if (ifs.eof()) {
            break;
        }
        msgpack.push_back(buffer);
    }
    ifs.close();

    // 3. parse into JSON

    const auto json = nlohmann::json::from_msgpack(msgpack);

    // 4. load database

    // add the new cameras to 'cam_db'
    const auto json_cameras = json.at("cameras");
    cam_db_->from_json(json_cameras);

    // add the new orb_params to 'orb_params_db'
    const auto json_orb_params = json.at("orb_params");
    orb_params_db_->from_json(json_orb_params);

    // shift keyframes and landmarks ids to not collide with first map loaded
    const auto json_keyfrms = json.at("keyframes");
    const auto json_landmarks = json.at("landmarks");
    
    // create temporaly json to sotre the new values
    nlohmann::json tmp_json_keyframes;
    nlohmann::json tmp_json_landmarks;

    // Translation anb rotation
    Mat33_t rotation_matrix = transf_matrix.block<3,3>(0,0);
    Vec3_t translation = transf_matrix.block<3,1>(0,3);

    // change values of keyframes
    for (auto& [keyfrm_id, json_keyfrm] : json_keyfrms.items()) {
        nlohmann::json tmp_json_kyfrm;

        // Transform keyframes
        if (json_keyfrm.contains("rot_cw")) {
            Mat33_t rot_wc = openvslam::data::convert_json_to_rotation(json_keyfrm["rot_cw"]).transpose();
            Vec3_t trans_wc = (-rot_wc) * openvslam::data::convert_json_to_translation(json_keyfrm["trans_cw"]);
            Vec3_t new_position = (scale_factor * rotation_matrix) * trans_wc + translation;
            nlohmann::json new_rot_cw =  openvslam::data::convert_rotation_to_json((rotation_matrix * rot_wc).transpose());
            tmp_json_kyfrm["rot_cw"] = std::move(new_rot_cw);
            nlohmann::json new_trans_cw =  openvslam::data::convert_translation_to_json((-rotation_matrix * rot_wc).inverse() * new_position);
            tmp_json_kyfrm["trans_cw"] = std::move(new_trans_cw);
        }
        else {
            tmp_json_kyfrm["trans_cw"] = std::move(json_keyfrm["trans_cw"]);
            tmp_json_kyfrm["rot_cw"] = std::move(json_keyfrm["rot_cw"]);
        }
        // shift ids of landmarks associated to keyframe
        std::vector<int> new_landmarks_ids;
        new_landmarks_ids.reserve(json_keyfrm["lm_ids"].size());
        for (auto& landmark_id : json_keyfrm["lm_ids"].get<std::vector<int>>()) {
            if (landmark_id != -1) {
                new_landmarks_ids.push_back(landmark_id + data::landmark::next_id_);
            } else {
                new_landmarks_ids.push_back(landmark_id);
            }
        }
        tmp_json_kyfrm["lm_ids"] = std::move(new_landmarks_ids);

        // shift id of frame associated to keyframe
        tmp_json_kyfrm["src_frm_id"] = json_keyfrm["src_frm_id"].get<unsigned int>() + data::frame::next_id_;

        // shift id of 'span_parent'
        tmp_json_kyfrm["span_parent"] = json_keyfrm["span_parent"].get<int>() + data::keyframe::next_id_;

        // shift ids of 'span_children'
        std::vector<int> new_children_ids;
        new_children_ids.reserve(json_keyfrm["span_children"].size());
        for (const auto& keyframe_id : json_keyfrm["span_children"].get<std::vector<int>>()) {
            new_children_ids.push_back(keyframe_id + data::keyframe::next_id_);
        }
        tmp_json_kyfrm["span_children"] = std::move(new_children_ids);

        // shift ids of 'loop_edges'
        std::vector<int> new_loop_edges_ids;
        new_loop_edges_ids.reserve(json_keyfrm["loop_edges"].size());
        for (const auto& loop_edge_id : json_keyfrm["loop_edges"].get<std::vector<int>>()) {
            new_loop_edges_ids.push_back(loop_edge_id + data::keyframe::next_id_);
        }
        tmp_json_kyfrm["loop_edges"] = std::move(new_loop_edges_ids);

        // copy other values unchanged
        tmp_json_kyfrm["cam"] = std::move(json_keyfrm["cam"]);
        tmp_json_kyfrm["depth_thr"] = std::move(json_keyfrm["depth_thr"]);
        tmp_json_kyfrm["depths"] = std::move(json_keyfrm["depths"]);
        tmp_json_kyfrm["descs"] = std::move(json_keyfrm["descs"]);
        tmp_json_kyfrm["keypts"] = std::move(json_keyfrm["keypts"]);
        tmp_json_kyfrm["n_keypts"] = std::move(json_keyfrm["n_keypts"]);
        tmp_json_kyfrm["n_scale_levels"] = std::move(json_keyfrm["n_scale_levels"]);
        tmp_json_kyfrm["orb_params"] = std::move(json_keyfrm["orb_params"]);
        tmp_json_kyfrm["scale_factor"] = std::move(json_keyfrm["scale_factor"]);
        tmp_json_kyfrm["ts"] = std::move(json_keyfrm["ts"]);
        tmp_json_kyfrm["undists"] = std::move(json_keyfrm["undists"]);
        tmp_json_kyfrm["x_rights"] = std::move(json_keyfrm["x_rights"]);

        // finally, move modified keyframe value to temporal json with the new id
        tmp_json_keyframes[std::to_string(std::stoi(keyfrm_id) + data::keyframe::next_id_)] = std::move(tmp_json_kyfrm);
    }

    // change values of landmarks
    for (auto& [landmark_id, json_landmark] : json_landmarks.items()) {
        nlohmann::json tmp_json_ldmrk;

        // transform the position in world
        tmp_json_ldmrk["pos_w"] = std::move(openvslam::data::convert_translation_to_json((scale_factor * rotation_matrix)*openvslam::data::convert_json_to_translation(json_landmark["pos_w"]) + translation));

        // shift 'ref_keyfrm' id
        tmp_json_ldmrk["ref_keyfrm"] = json_landmark["ref_keyfrm"].get<int>() + data::keyframe::next_id_;

        // shift '1st_keyfrm' id
        tmp_json_ldmrk["1st_keyfrm"] = json_landmark["1st_keyfrm"].get<int>() + data::keyframe::next_id_;

        // copy other values unchanged
        tmp_json_ldmrk["n_vis"] = std::move(json_landmark.at("n_vis"));
        tmp_json_ldmrk["n_fnd"] = std::move(json_landmark.at("n_fnd"));

        // finally, move modified landmark value to temporal json with the new id
        tmp_json_landmarks[std::to_string(std::stoi(landmark_id) + data::landmark::next_id_)] = std::move(tmp_json_ldmrk);
    }

    // increase static variables
    data::frame::next_id_    += json.at("frame_next_id").get<unsigned int>();
    data::keyframe::next_id_ += json.at("keyframe_next_id").get<unsigned int>();
    data::landmark::next_id_ += json.at("landmark_next_id").get<unsigned int>();
    
    // add to database with the new keyfrms and landmarks
    map_db_->add_from_json(cam_db_, orb_params_db_, bow_vocab_, tmp_json_keyframes, tmp_json_landmarks);
    const auto keyfrms = map_db_->get_all_keyframes();
    for (const auto keyfrm : keyfrms) {
        bow_db_->add_keyframe(keyfrm);

    }
}


} // namespace io
} // namespace openvslam
