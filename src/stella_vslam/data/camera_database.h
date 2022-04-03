#ifndef STELLA_VSLAM_DATA_CAMERA_DATABASE_H
#define STELLA_VSLAM_DATA_CAMERA_DATABASE_H

#include <mutex>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

typedef struct sqlite3 sqlite3;

namespace stella_vslam {

namespace camera {
class base;
} // namespace camera

namespace data {

class camera_database {
public:
    explicit camera_database(camera::base* curr_camera);

    ~camera_database();

    camera::base* get_camera(const std::string& camera_name) const;

    void from_json(const nlohmann::json& json_cameras);

    nlohmann::json to_json() const;

    bool from_db(sqlite3* db);

    bool to_db(sqlite3* db) const;

private:
    //-----------------------------------------
    //! mutex to access the database
    mutable std::mutex mtx_database_;
    //! pointer to the camera which used in the current tracking
    //! (NOTE: the object is owned by config class,
    //!  thus this class does NOT delete the object of curr_camera_)
    camera::base* curr_camera_ = nullptr;
    //! database (key: camera name, value: pointer of camera::base)
    //! (NOTE: tracking camera must NOT be contained in the database)
    std::unordered_map<std::string, camera::base*> database_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_CAMERA_DATABASE_H
