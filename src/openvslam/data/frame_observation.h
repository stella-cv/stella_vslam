#ifndef OPENVSLAM_DATA_FRAME_OBSERVATION_H
#define OPENVSLAM_DATA_FRAME_OBSERVATION_H

#include "openvslam/type.h"

namespace openvslam {
namespace data {

struct frame_observation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame_observation() = default;
    frame_observation(unsigned int num_keypts, const std::vector<cv::KeyPoint>& keypts, const cv::Mat& descriptors,
                      const std::vector<cv::KeyPoint>& undist_keypts, const eigen_alloc_vector<Vec3_t>& bearings,
                      const std::vector<float>& stereo_x_right, const std::vector<float>& depths,
                      const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells)
        : num_keypts_(num_keypts), keypts_(keypts), descriptors_(descriptors), undist_keypts_(undist_keypts), bearings_(bearings),
          stereo_x_right_(stereo_x_right), depths_(depths), keypt_indices_in_cells_(keypt_indices_in_cells) {}

    //! number of keypoints
    unsigned int num_keypts_ = 0;
    //! keypoints of monocular or stereo left image
    std::vector<cv::KeyPoint> keypts_;
    //! descriptors
    cv::Mat descriptors_;
    //! undistorted keypoints of monocular or stereo left image
    std::vector<cv::KeyPoint> undist_keypts_;
    //! bearing vectors
    eigen_alloc_vector<Vec3_t> bearings_;
    //! disparities
    std::vector<float> stereo_x_right_;
    //! depths
    std::vector<float> depths_;
    //! keypoint indices in each of the cells
    std::vector<std::vector<std::vector<unsigned int>>> keypt_indices_in_cells_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_FRAME_OBSERVATION_H
