#ifndef STELLA_VSLAM_MARKER_MODEL_ACURONANO_H
#define STELLA_VSLAM_MARKER_MODEL_ACURONANO_H

#include "stella_vslam/type.h"
#include "stella_vslam/marker_model/base.h"

#include <string>
#include <limits>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json_fwd.hpp>

namespace stella_vslam {
namespace marker_model {

class aruconano : public marker_model::base {
public:
    //! Constructor
    aruconano(double width, int dict);

    //! Destructor
    virtual ~aruconano();

    int dict_;

    //! Encode marker_model information as JSON
    virtual nlohmann::json to_json() const;
};

std::ostream& operator<<(std::ostream& os, const aruconano& params);

} // namespace marker_model
} // namespace stella_vslam

#endif // STELLA_VSLAM_MARKER_MODEL_ACURONANO_H
