#include "stella_vslam/marker_detector/aruco.h"
#include "stella_vslam/marker_model/aruco.h"
#include "stella_vslam/data/marker2d.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/solve/pnp_solver.h"

#include <opencv2/aruco.hpp>

namespace {
cv::Ptr<cv::aruco::Dictionary> get_predefined_dictionary(int marker_size, int max_markers) {
    cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    if (marker_size == 4 && max_markers == 50) {
        dictionary_name = cv::aruco::DICT_4X4_50;
    }
    else if (marker_size == 4 && max_markers == 100) {
        dictionary_name = cv::aruco::DICT_4X4_100;
    }
    else if (marker_size == 4 && max_markers == 250) {
        dictionary_name = cv::aruco::DICT_4X4_250;
    }
    else if (marker_size == 4 && max_markers == 1000) {
        dictionary_name = cv::aruco::DICT_4X4_1000;
    }
    else if (marker_size == 5 && max_markers == 50) {
        dictionary_name = cv::aruco::DICT_5X5_50;
    }
    else if (marker_size == 5 && max_markers == 100) {
        dictionary_name = cv::aruco::DICT_5X5_100;
    }
    else if (marker_size == 5 && max_markers == 250) {
        dictionary_name = cv::aruco::DICT_5X5_250;
    }
    else if (marker_size == 5 && max_markers == 1000) {
        dictionary_name = cv::aruco::DICT_5X5_1000;
    }
    else if (marker_size == 6 && max_markers == 50) {
        dictionary_name = cv::aruco::DICT_6X6_50;
    }
    else if (marker_size == 6 && max_markers == 100) {
        dictionary_name = cv::aruco::DICT_6X6_100;
    }
    else if (marker_size == 6 && max_markers == 250) {
        dictionary_name = cv::aruco::DICT_6X6_250;
    }
    else if (marker_size == 6 && max_markers == 1000) {
        dictionary_name = cv::aruco::DICT_6X6_1000;
    }
    else if (marker_size == 7 && max_markers == 50) {
        dictionary_name = cv::aruco::DICT_7X7_50;
    }
    else if (marker_size == 7 && max_markers == 100) {
        dictionary_name = cv::aruco::DICT_7X7_100;
    }
    else if (marker_size == 7 && max_markers == 250) {
        dictionary_name = cv::aruco::DICT_7X7_250;
    }
    else if (marker_size == 7 && max_markers == 1000) {
        dictionary_name = cv::aruco::DICT_7X7_1000;
    }
    else {
        throw std::runtime_error("Invalid predefined dictionary marker_size/max_markers");
    }
    return cv::aruco::getPredefinedDictionary(dictionary_name);
}
} // namespace

namespace stella_vslam {
namespace marker_detector {
aruco::aruco(const camera::base* camera,
             const std::shared_ptr<marker_model::base>& marker_model,
             const unsigned int num_iter,
             const double reproj_error_threshold)
    : base(camera, marker_model, num_iter, reproj_error_threshold) {
    parameters_ = cv::aruco::DetectorParameters::create();
    auto aruco_marker_model = std::static_pointer_cast<marker_model::aruco>(marker_model);
    dictionary_ = get_predefined_dictionary(aruco_marker_model->marker_size_, aruco_marker_model->max_markers_);
}

bool aruco::is_valid() {
    return true;
}

void aruco::detect_2d(const cv::_InputArray& in_image, std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids) const {
    cv::aruco::detectMarkers(in_image, dictionary_, corners, ids, parameters_);
}

} // namespace marker_detector
} // namespace stella_vslam
