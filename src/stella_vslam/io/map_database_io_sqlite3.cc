#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/camera_database.h"
#include "stella_vslam/data/bow_database.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/io/map_database_io_sqlite3.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <fstream>

namespace stella_vslam {
namespace io {

void map_database_io_sqlite3::save(const std::string& path,
                                   const data::camera_database* const cam_db,
                                   const data::orb_params_database* const orb_params_db,
                                   const data::map_database* const map_db) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    assert(cam_db && map_db);

    // Open database
    sqlite3* db = nullptr;
    int ret = sqlite3_open(path.c_str(), &db);
    if (ret != SQLITE_OK) {
        spdlog::error("Failed to open SQL database");
        return;
    }

    // Write data into database
    bool ok = save_stats(db, map_db);
    ok = ok && cam_db->to_db(db);
    ok = ok && map_db->to_db(db);

    sqlite3_close(db);
    if (ok) {
        spdlog::info("Save the map database to {}", path);
    }
    else {
        spdlog::info("Failed save the map database");
    }
}

void map_database_io_sqlite3::load(const std::string& path,
                                   data::camera_database* cam_db,
                                   data::orb_params_database* orb_params_db,
                                   data::map_database* map_db,
                                   data::bow_database* bow_db,
                                   data::bow_vocabulary* bow_vocab) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);
    assert(cam_db && map_db && bow_db && bow_vocab);

    // Open database
    sqlite3* db = nullptr;
    int ret = sqlite3_open(path.c_str(), &db);
    if (ret != SQLITE_OK) {
        spdlog::error("Failed to open SQL database");
        return;
    }

    // load from database
    bool ok = cam_db->from_db(db);
    ok = ok && map_db->from_db(db, cam_db, orb_params_db, bow_vocab);
    ok = ok && load_stats(db, map_db);

    // update bow database
    if (ok) {
        const auto keyfrms = map_db->get_all_keyframes();
        for (const auto& keyfrm : keyfrms) {
            bow_db->add_keyframe(keyfrm);
        }
    }

    sqlite3_close(db);
}

bool map_database_io_sqlite3::save_stats(sqlite3* db, const data::map_database* map_db) const {
    int ret = sqlite3_exec(db, "DROP TABLE IF EXISTS stats;", nullptr, nullptr, nullptr);
    if (ret == SQLITE_OK) {
        ret = sqlite3_exec(db, "CREATE TABLE stats(id INTEGER PRIMARY KEY, frame_next_id INTEGER, keyframe_next_id INTEGER, landmark_next_id INTEGER);", nullptr, nullptr, nullptr);
    }
    if (ret == SQLITE_OK) {
        ret = sqlite3_exec(db, "BEGIN;", nullptr, nullptr, nullptr);
    }
    sqlite3_stmt* stmt = nullptr;
    if (ret == SQLITE_OK) {
        ret = sqlite3_prepare_v2(db, "INSERT INTO stats(id, frame_next_id, keyframe_next_id, landmark_next_id) VALUES(?, ?, ?, ?)", -1, &stmt, nullptr);
    }
    if (ret != SQLITE_OK) {
        spdlog::error("SQLite error: {}", sqlite3_errmsg(db));
        return false;
    }
    if (ret == SQLITE_OK) {
        ret = sqlite3_bind_int64(stmt, 1, 0);
    }
    if (ret == SQLITE_OK) {
        ret = sqlite3_bind_int64(stmt, 3, map_db->next_keyframe_id_);
    }
    if (ret == SQLITE_OK) {
        ret = sqlite3_bind_int64(stmt, 4, map_db->next_landmark_id_);
    }
    if (ret != SQLITE_OK) {
        spdlog::error("SQLite error: {}", sqlite3_errmsg(db));
        return false;
    }
    if (ret == SQLITE_OK) {
        ret = sqlite3_step(stmt);
    }
    if (ret != SQLITE_DONE) {
        spdlog::error("SQLite step is not done: {}", sqlite3_errmsg(db));
    }
    sqlite3_finalize(stmt);
    if (ret == SQLITE_DONE) {
        ret = sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
    }
    if (ret != SQLITE_OK) {
        spdlog::error("SQLite error: {}", sqlite3_errmsg(db));
        return false;
    }
    else {
        return true;
    }
}

bool map_database_io_sqlite3::load_stats(sqlite3* db, data::map_database* map_db) const {
    sqlite3_stmt* stmt;
    int ret = sqlite3_prepare_v2(db, "SELECT * FROM stats;", -1, &stmt, nullptr);
    if (ret != SQLITE_OK) {
        spdlog::error("SQLite error: {}", sqlite3_errmsg(db));
        return false;
    }

    ret = sqlite3_step(stmt);
    map_db->next_keyframe_id_ = sqlite3_column_int64(stmt, 2);
    map_db->next_landmark_id_ = sqlite3_column_int64(stmt, 3);
    sqlite3_finalize(stmt);
    return ret == SQLITE_ROW;
}

} // namespace io
} // namespace stella_vslam
