#ifndef OPENVSLAM_IO_TRAJECTORY_IO_H
#define OPENVSLAM_IO_TRAJECTORY_IO_H

#include "openvslam/platform.h"

#include <string>

namespace openvslam {

namespace data {
class map_database;
} // namespace data

namespace io {

class DECLSPEC trajectory_io {
public:
    /**
     * Constructor
     */
    explicit trajectory_io(data::map_database* map_db);

    /**
     * Destructor
     */
    ~trajectory_io() = default;

    /**
     * Save the frame trajectory in the specified format
     */
    void save_frame_trajectory(const std::string& path, const std::string& format) const;

    /**
     * Save the keyframe trajectory in the specified format
     */
    void save_keyframe_trajectory(const std::string& path, const std::string& format) const;

private:
    //! map_database
    data::map_database* const map_db_ = nullptr;
};

} // namespace io
} // namespace openvslam

#endif // OPENVSLAM_IO_TRAJECTORY_IO_H
