#include "stella_vslam/util/angle.h"

namespace stella_vslam {
namespace util {
namespace angle {

float diff(float angle1, float angle2) {
    float ret = angle1 - angle2;
    if (ret <= -180.0) {
        ret += 360.0;
    }
    if (ret > 180.0) {
        ret -= 360.0;
    }
    return ret;
}
} // namespace angle
} // namespace util
} // namespace stella_vslam
