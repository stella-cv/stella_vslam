#ifndef STELLA_VSLAM_PUBLISH_FRAME_PUBLISHER_H
#define STELLA_VSLAM_PUBLISH_FRAME_PUBLISHER_H

#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"

#include <mutex>
#include <vector>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace stella_vslam {

class tracking_module;

namespace data {
class map_database;
} // namespace data

namespace publish {

class frame_publisher {
public:
    /**
     * Constructor
     */
    frame_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db,
                    const unsigned int img_width = 1024);

    /**
     * Destructor
     */
    virtual ~frame_publisher();

    /**
     * Update tracking information
     * NOTE: should be accessed from system thread
     */
    void update(const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                bool mapping_is_enabled,
                tracker_state_t tracking_state,
                std::vector<cv::KeyPoint>& keypts,
                const cv::Mat& img,
                double elapsed_ms);

    /**
     * Get the current image with tracking information
     * NOTE: should be accessed from viewer thread
     */
    cv::Mat draw_frame();

protected:
    unsigned int draw_tracked_points(cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypts,
                                     const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                                     const bool mapping_is_enabled,
                                     const float mag = 1.0) const;

    // colors (BGR)
    const cv::Scalar mapping_color_{0, 255, 255};
    const cv::Scalar localization_color_{255, 255, 0};

    //! config
    std::shared_ptr<config> cfg_;
    //! map database
    data::map_database* map_db_;
    //! maximum size of output images
    const int img_width_;

    // -------------------------------------------
    //! mutex to access variables below
    std::mutex mtx_;

    //! raw img
    cv::Mat img_;
    //! tracking state
    tracker_state_t tracking_state_;

    //! current keypoints
    std::vector<cv::KeyPoint> curr_keypts_;

    //! elapsed time for tracking
    double elapsed_ms_ = 0.0;

    //! mapping module status
    bool mapping_is_enabled_;

    std::vector<std::shared_ptr<data::landmark>> curr_lms_;
};

} // namespace publish
} // namespace stella_vslam

#endif // STELLA_VSLAM_PUBLISH_FRAME_PUBLISHER_H
