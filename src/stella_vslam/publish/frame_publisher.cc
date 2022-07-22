#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/publish/frame_publisher.h"

#include <iomanip>

#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <tinycolormap.hpp>

namespace stella_vslam {
namespace publish {

frame_publisher::frame_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db,
                                 const unsigned int img_width)
    : cfg_(cfg), map_db_(map_db), img_width_(img_width),
      img_(cv::Mat(480, img_width_, CV_8UC3, cv::Scalar(0, 0, 0))) {
    spdlog::debug("CONSTRUCT: publish::frame_publisher");
}

frame_publisher::~frame_publisher() {
    spdlog::debug("DESTRUCT: publish::frame_publisher");
}

cv::Mat frame_publisher::draw_frame() {
    cv::Mat img;
    tracker_state_t tracking_state;
    std::vector<cv::KeyPoint> curr_keypts;
    bool mapping_is_enabled;
    std::vector<std::shared_ptr<data::landmark>> curr_lms;

    // copy to avoid memory access conflict
    {
        std::lock_guard<std::mutex> lock(mtx_);

        img_.copyTo(img);

        tracking_state = tracking_state_;

        // copy tracking information
        curr_keypts = curr_keypts_;

        mapping_is_enabled = mapping_is_enabled_;

        curr_lms = curr_lms_;
    }

    // resize image
    const float mag = (img_width_ < img_.cols) ? static_cast<float>(img_width_) / img.cols : 1.0;
    if (mag != 1.0) {
        cv::resize(img, img, cv::Size(), mag, mag, cv::INTER_NEAREST);
    }

    // to draw COLOR information
    if (img.channels() < 3) {
        cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    // draw keypoints
    unsigned int num_tracked = 0;
    switch (tracking_state) {
        case tracker_state_t::Tracking: {
            num_tracked = draw_tracked_points(img, curr_keypts, curr_lms, mapping_is_enabled, mag);
            break;
        }
        default: {
            break;
        }
    }

    spdlog::trace("num_tracked: {}", num_tracked);

    return img;
}

unsigned int frame_publisher::draw_tracked_points(cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypts,
                                                  const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                                                  const bool mapping_is_enabled,
                                                  const float mag) const {
    constexpr float radius = 5;

    unsigned int num_tracked = 0;

    for (unsigned int i = 0; i < curr_keypts.size(); ++i) {
        const auto& lm = curr_lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        const cv::Point2f pt_begin{curr_keypts.at(i).pt.x * mag - radius, curr_keypts.at(i).pt.y * mag - radius};
        const cv::Point2f pt_end{curr_keypts.at(i).pt.x * mag + radius, curr_keypts.at(i).pt.y * mag + radius};

        double score = lm->get_observed_ratio();
        const tinycolormap::Color color = tinycolormap::GetColor(score, tinycolormap::ColormapType::Turbo);
        if (mapping_is_enabled) {
            const cv::Scalar mapping_color{color.b() * 255, color.g() * 255, color.r() * 255};
            cv::circle(img, curr_keypts.at(i).pt * mag, 2, mapping_color, -1);
        }
        else {
            cv::circle(img, curr_keypts.at(i).pt * mag, 2, localization_color_, -1);
        }

        ++num_tracked;
    }

    return num_tracked;
}

void frame_publisher::update(const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                             bool mapping_is_enabled,
                             tracker_state_t tracking_state,
                             std::vector<cv::KeyPoint>& keypts,
                             const cv::Mat& img,
                             double elapsed_ms) {
    std::lock_guard<std::mutex> lock(mtx_);

    img.copyTo(img_);

    curr_keypts_ = keypts;
    elapsed_ms_ = elapsed_ms;
    mapping_is_enabled_ = mapping_is_enabled;
    tracking_state_ = tracking_state;
    curr_lms_ = curr_lms;
}

} // namespace publish
} // namespace stella_vslam
