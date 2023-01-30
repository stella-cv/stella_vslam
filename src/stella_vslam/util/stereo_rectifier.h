#ifndef STELLA_VSLAM_UTIL_STEREO_RECTIFIER_H
#define STELLA_VSLAM_UTIL_STEREO_RECTIFIER_H

#include "stella_vslam/config.h"
#include "stella_vslam/camera/base.h"

#include <memory>

#include <opencv2/core/mat.hpp>

namespace stella_vslam {
namespace util {

class stereo_rectifier {
public:
    //! Constructor
    explicit stereo_rectifier(const std::shared_ptr<config>& cfg, camera::base* camera);

    //! Constructor
    stereo_rectifier(camera::base* camera, const YAML::Node& yaml_node);

    //! Destructor
    virtual ~stereo_rectifier();

    //! Apply stereo-rectification
    void rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
                 cv::Mat& out_img_l, cv::Mat& out_img_r) const;

private:
    //! Parse std::vector as cv::Mat
    static cv::Mat parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec);

    //! Load model type before rectification from YAML
    static camera::model_type_t load_model_type(const YAML::Node& yaml_node);

    //! camera model type before rectification
    const camera::model_type_t model_type_;

    //! undistortion map for x-axis in left image
    cv::Mat undist_map_x_l_;
    //! undistortion map for y-axis in left image
    cv::Mat undist_map_y_l_;
    //! undistortion map for x-axis in right image
    cv::Mat undist_map_x_r_;
    //! undistortion map for y-axis in right image
    cv::Mat undist_map_y_r_;
};

} // namespace util
} // namespace stella_vslam

#endif // STELLA_VSLAM_UTIL_STEREO_RECTIFIER_H
