#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/camera_database.h"
#include "stella_vslam/data/orb_params_database.h"
#include "stella_vslam/data/bow_database.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/io/map_database_io_msgpack.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <fstream>

namespace stella_vslam {
namespace io {

bool map_database_io_msgpack::save(const std::string& path,
                                   const data::camera_database* const cam_db,
                                   const data::orb_params_database* const orb_params_db,
                                   const data::map_database* const map_db) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    assert(cam_db && orb_params_db && map_db);
    const auto cameras = cam_db->to_json();
    const auto orb_params = orb_params_db->to_json();
    nlohmann::json keyfrms;
    nlohmann::json landmarks;
    map_db->to_json(keyfrms, landmarks);

    nlohmann::json json{{"cameras", cameras},
                        {"orb_params", orb_params},
                        {"keyframes", keyfrms},
                        {"landmarks", landmarks},
                        {"keyframe_next_id", static_cast<unsigned int>(map_db->next_keyframe_id_)},
                        {"landmark_next_id", static_cast<unsigned int>(map_db->next_landmark_id_)}};

    std::ofstream ofs(path, std::ios::out | std::ios::binary);

    if (ofs.is_open()) {
        spdlog::info("save the MessagePack file of database to {}", path);
        const auto msgpack = nlohmann::json::to_msgpack(json);
        ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
        ofs.close();
        return true;
    }
    else {
        spdlog::critical("cannot create a file at {}", path);
        return false;
    }
}

bool map_database_io_msgpack::load(const std::string& path,
                                   data::camera_database* cam_db,
                                   data::orb_params_database* orb_params_db,
                                   data::map_database* map_db,
                                   data::bow_database* bow_db,
                                   data::bow_vocabulary* bow_vocab) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);
    assert(cam_db && orb_params_db && map_db && bow_db && bow_vocab);

    // load binary bytes

    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        spdlog::critical("cannot load the file at {}", path);
        return false;
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

    // parse into JSON

    const auto json = nlohmann::json::from_msgpack(msgpack);

    // load database
    const auto json_cameras = json.at("cameras");
    cam_db->from_json(json_cameras);
    const auto json_orb_params = json.at("orb_params");
    orb_params_db->from_json(json_orb_params);
    const auto json_keyfrms = json.at("keyframes");
    const auto json_landmarks = json.at("landmarks");
    map_db->from_json(cam_db, orb_params_db, bow_vocab, json_keyfrms, json_landmarks);
    // load next ID
    map_db->next_keyframe_id_ += json.at("keyframe_next_id").get<unsigned int>();
    map_db->next_landmark_id_ += json.at("landmark_next_id").get<unsigned int>();

    // update bow database
    const auto keyfrms = map_db->get_all_keyframes();
    for (const auto& keyfrm : keyfrms) {
        bow_db->add_keyframe(keyfrm);
    }
    return true;
}

} // namespace io
} // namespace stella_vslam
