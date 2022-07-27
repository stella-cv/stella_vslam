#ifndef SOCKET_PUBLISHER_DATA_SERIALIZER_H
#define SOCKET_PUBLISHER_DATA_SERIALIZER_H

#include "stella_vslam/type.h"

#include <memory>

#include <Eigen/Core>
#include <sioclient/sio_client.h>

namespace stella_vslam {

class config;

namespace data {
class keyframe;
class landmark;
} // namespace data

namespace publish {
class frame_publisher;
class map_publisher;
} // namespace publish

} // namespace stella_vslam

namespace socket_publisher {

class data_serializer {
public:
    data_serializer(const std::shared_ptr<stella_vslam::publish::frame_publisher>& frame_publisher,
                    const std::shared_ptr<stella_vslam::publish::map_publisher>& map_publisher,
                    bool publish_points);

    std::string serialize_messages(const std::vector<std::string>& tags, const std::vector<std::string>& messages);

    std::string send_full_map();

    std::string serialize_map_diff();

    std::string serialize_latest_frame(const unsigned int image_quality_);

    static std::string serialized_reset_signal_;

private:
    const std::shared_ptr<stella_vslam::publish::frame_publisher> frame_publisher_;
    const std::shared_ptr<stella_vslam::publish::map_publisher> map_publisher_;
    bool publish_points_ = true;
    std::unique_ptr<std::unordered_map<unsigned int, double>> keyframe_hash_map_;
    std::unique_ptr<std::unordered_map<unsigned int, double>> point_hash_map_;

    double current_pose_hash_ = 0;
    int frame_hash_ = 0;

    inline double get_vec_hash(const stella_vslam::Vec3_t& point) {
        return point[0] + point[1] + point[2];
    }

    inline double get_mat_hash(const stella_vslam::Mat44_t& pose) {
        return pose(0, 3) + pose(1, 3) + pose(2, 3);
    }

    std::string serialize_as_protobuf(const std::vector<std::shared_ptr<stella_vslam::data::keyframe>>& keyfrms,
                                      const std::vector<std::shared_ptr<stella_vslam::data::landmark>>& all_landmarks,
                                      const std::set<std::shared_ptr<stella_vslam::data::landmark>>& local_landmarks,
                                      const stella_vslam::Mat44_t& current_camera_pose);

    std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
};

} // namespace socket_publisher

#endif // SOCKET_PUBLISHER_DATA_SERIALIZER_H
