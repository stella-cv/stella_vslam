#ifndef OPENVSLAM_MATCH_ROBUST_H
#define OPENVSLAM_MATCH_ROBUST_H

#include "openvslam/type.h"
#include "openvslam/match/base.h"

namespace openvslam {

namespace data {
class frame;
class keyframe;
class landmark;
} // namespace data

namespace match {

class robust final : public base {
public:
    explicit robust(const float lowe_ratio, const bool check_orientation)
        : base(lowe_ratio, check_orientation) {}

    ~robust() final = default;

    unsigned int match_for_triangulation(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2, const Mat33_t& E_12,
                                         std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs) const;

    unsigned int match_frame_and_keyframe(data::frame& frm, data::keyframe* keyfrm,
                                          std::vector<data::landmark*>& matched_lms_in_frm) const;

    unsigned int brute_force_match(data::frame& frm, data::keyframe* keyfrm, std::vector<std::pair<int, int>>& matches) const;

private:
    bool check_epipolar_constraint(const Vec3_t& bearing_1, const Vec3_t& bearing_2,
                                   const Mat33_t& E_12, const float bearing_1_scale_factor = 1.0) const;
};

} // namespace match
} // namespace openvslam

#endif // OPENVSLAM_MATCH_ROBUST_H
