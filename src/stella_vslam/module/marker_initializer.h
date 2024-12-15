#ifndef STELLA_VSLAM_MODULE_MARKER_INITIALIZER_H
#define STELLA_VSLAM_MODULE_MARKER_INITIALIZER_H

#include "stella_vslam/type.h"

namespace stella_vslam {

namespace data {
class marker;
} // namespace data

namespace module {

class marker_initializer {
public:
    static void check_marker_initialization(data::marker& mkr, size_t needed_observations_for_initialization);

private:
    const size_t required_keyframes_for_marker_initialization_ = 3;
};

} // namespace module
} // namespace stella_vslam

#endif // STELLA_VSLAM_MODULE_MARKER_INITIALIZER_H
