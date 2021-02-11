#include <openvslam_ros.h>

#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <openvslam/publish/map_publisher.h>
#include <Eigen/Geometry>

namespace openvslam_ros {
system::system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : SLAM_(cfg, vocab_file_path), cfg_(cfg), private_nh_("~"), it_(nh_), tp_0_(std::chrono::steady_clock::now()),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(private_nh_.advertise<nav_msgs::Odometry>("camera_pose", 1)) {}

void system::publish_pose() {
    // SLAM get the motion matrix publisher
    auto cam_pose_wc = SLAM_.get_map_publisher()->get_current_cam_pose_wc();

    // Extract rotation matrix and translation vector from 
    Eigen::Matrix3d rot = cam_pose_wc.block<3, 3>(0, 0);
    Eigen::Vector3d trans = cam_pose_wc.block<3, 1>(0, 3);
    Eigen::Matrix3d cv_to_ros;
    cv_to_ros <<
        0, 0, 1,
        -1, 0, 0,
        0,-1, 0;

    // Transform from CV coordinate system to ROS coordinate system on camera coordinates
    Eigen::Quaterniond quat(cv_to_ros * rot * cv_to_ros.transpose());
    trans = cv_to_ros * trans;

    // Create odometry message and update it with current camera pose
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.child_frame_id = "camera_link";
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();
    pose_msg.pose.pose.position.x = trans(0);
    pose_msg.pose.pose.position.y = trans(1);
    pose_msg.pose.pose.position.z = trans(2);
    pose_pub_.publish(pose_msg);
}

mono::mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    sub_ = it_.subscribe("camera/image_raw", 1, &mono::callback, this);
}
void mono::callback(const sensor_msgs::ImageConstPtr& msg) {
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    SLAM_.feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
}

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
               const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path),
      rectifier_(rectify ? std::make_shared<openvslam::util::stereo_rectifier>(cfg) : nullptr),
      left_sf_(it_, "camera/left/image_raw", 1),
      right_sf_(it_, "camera/right/image_raw", 1),
      sync_(SyncPolicy(10), left_sf_, right_sf_) {
    sync_.registerCallback(&stereo::callback, this);
}

void stereo::callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right) {
    auto leftcv = cv_bridge::toCvShare(left)->image;
    auto rightcv = cv_bridge::toCvShare(right)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    SLAM_.feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
}

rgbd::rgbd(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path),
      color_sf_(it_, "camera/color/image_raw", 1),
      depth_sf_(it_, "camera/depth/image_raw", 1),
      sync_(SyncPolicy(10), color_sf_, depth_sf_) {
    sync_.registerCallback(&rgbd::callback, this);
}

void rgbd::callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth) {
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    SLAM_.feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
}
} // namespace openvslam_ros
