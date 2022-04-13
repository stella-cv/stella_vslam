#ifndef STELLA_VSLAM_MARKER_MODEL_ACURO_H
#define STELLA_VSLAM_MARKER_MODEL_ACURO_H

#include "stella_vslam/type.h"
#include "stella_vslam/marker_model/base.h"

#include <string>
#include <limits>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json_fwd.hpp>

namespace stella_vslam {
namespace marker_model {

class aruco : public marker_model::base {
public:
    //! Constructor
    aruco(double width, int marker_size, int max_markers);

    //! Destructor
    virtual ~aruco();

    //! marker definition
    int marker_size_;
    int max_markers_;

    //! Encode marker_model information as JSON
    virtual nlohmann::json to_json() const;
};

std::ostream& operator<<(std::ostream& os, const aruco& params);

} // namespace marker_model
} // namespace stella_vslam

#endif // STELLA_VSLAM_MARKER_MODEL_ACURO_H
