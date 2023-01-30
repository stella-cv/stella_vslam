#ifndef STELLA_VSLAM_IO_MAP_DATABASE_IO_FACTORY_H
#define STELLA_VSLAM_IO_MAP_DATABASE_IO_FACTORY_H

#include "stella_vslam/data/bow_vocabulary.h"
#include "stella_vslam/io/map_database_io_base.h"
#include "stella_vslam/io/map_database_io_msgpack.h"
#include "stella_vslam/io/map_database_io_sqlite3.h"

#include <string>

namespace stella_vslam {

namespace data {
class camera_database;
class bow_database;
class map_database;
} // namespace data

namespace io {

class map_database_io_factory {
public:
    static std::shared_ptr<map_database_io_base> create(const std::string& map_format) {
        std::shared_ptr<map_database_io_base> map_database_io;
        if (map_format == "sqlite3") {
            map_database_io = std::make_shared<io::map_database_io_sqlite3>();
        }
        else if (map_format == "msgpack") {
            map_database_io = std::make_shared<io::map_database_io_msgpack>();
        }
        else {
            throw std::runtime_error("Invalid map format: " + map_format);
        }
        return map_database_io;
    }
};

} // namespace io
} // namespace stella_vslam

#endif // STELLA_VSLAM_IO_MAP_DATABASE_IO_FACTORY_H
