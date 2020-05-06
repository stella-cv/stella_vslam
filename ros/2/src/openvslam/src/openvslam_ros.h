#ifndef OPENVSLAM_ROS_H
#define OPENVSLAM_ROS_H

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/util/stereo_rectifier.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

namespace openvslam_ros {
class system {
public:
    system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    openvslam::system SLAM_;
    std::shared_ptr<openvslam::config> cfg_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    rmw_qos_profile_t custom_qos_;
    std::chrono::steady_clock::time_point tp_0_;
    cv::Mat mask_;
    std::vector<double> track_times_;
};

class mono : public system {
public:
    mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    image_transport::Subscriber sub_;
};
} // namespace openvslam_ros

#endif // OPENVSLAM_ROS_H
