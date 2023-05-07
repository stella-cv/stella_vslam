#include "pangolin_viewer/viewer.h"

#include "stella_vslam/config.h"
#include "stella_vslam/system.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/publish/frame_publisher.h"
#include "stella_vslam/publish/map_publisher.h"
#include "stella_vslam/util/yaml.h"

#include <opencv2/highgui.hpp>
#include <tinycolormap.hpp>

namespace {
int parse_int(const std::string& msg) {
    int ret = -1;
    try {
        ret = stoi(msg);
    }
    catch (std::invalid_argument& e) {
    }
    catch (std::out_of_range& e) {
    }
    return ret;
}
} // namespace

namespace pangolin_viewer {

viewer::viewer(const YAML::Node& yaml_node,
               const std::shared_ptr<stella_vslam::system>& system,
               const std::shared_ptr<stella_vslam::publish::frame_publisher>& frame_publisher,
               const std::shared_ptr<stella_vslam::publish::map_publisher>& map_publisher)
    : system_(system), frame_publisher_(frame_publisher), map_publisher_(map_publisher),
      interval_ms_(1000.0f / yaml_node["fps"].as<float>(30.0)),
      viewpoint_x_(yaml_node["viewpoint_x"].as<float>(0.0)),
      viewpoint_y_(yaml_node["viewpoint_y"].as<float>(-10.0)),
      viewpoint_z_(yaml_node["viewpoint_z"].as<float>(-0.1)),
      viewpoint_f_(yaml_node["viewpoint_f"].as<float>(2000.0)),
      keyfrm_size_(yaml_node["keyframe_size"].as<float>(0.1)),
      keyfrm_line_width_(yaml_node["keyframe_line_width"].as<unsigned int>(1)),
      graph_line_width_(yaml_node["graph_line_width"].as<unsigned int>(1)),
      point_size_(yaml_node["point_size"].as<unsigned int>(2)),
      camera_size_(yaml_node["camera_size"].as<float>(0.15)),
      camera_line_width_(yaml_node["camera_line_width"].as<unsigned int>(2)),
      menu_width_(yaml_node["menu_width"].as<unsigned int>(230)),
      cs_(yaml_node["color_scheme"].as<std::string>("black")),
      mapping_mode_(system->mapping_module_is_enabled()),
      loop_detection_mode_(system->loop_detector_is_enabled()) {}

void viewer::run() {
    is_terminated_ = false;

    pangolin::CreateWindowAndBind(map_viewer_name_, 1024, 768);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // depth testing to be enabled for 3D mouse handler
    glEnable(GL_DEPTH_TEST);

    // setup camera renderer
    s_cam_ = std::unique_ptr<pangolin::OpenGlRenderState>(new pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(map_viewer_width_, map_viewer_height_, viewpoint_f_, viewpoint_f_,
                                   map_viewer_width_ / 2, map_viewer_height_ / 2, 0.1, 1e6),
        pangolin::ModelViewLookAt(viewpoint_x_, viewpoint_y_, viewpoint_z_, 0, 0, 0, 0.0, -1.0, 0.0)));

    // create map window
    pangolin::View& d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -map_viewer_width_ / map_viewer_height_)
                                .SetHandler(new pangolin::Handler3D(*s_cam_));

    // create menu panel
    create_menu_panel();

    // create frame window
    cv::namedWindow(frame_viewer_name_);

    pangolin::OpenGlMatrix gl_cam_pose_wc;
    gl_cam_pose_wc.SetIdentity();

    while (true) {
        // clear buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 1. draw the map window

        // get current camera pose as OpenGL matrix
        gl_cam_pose_wc = get_current_cam_pose();

        // make the rendering camera follow to the current camera
        follow_camera(gl_cam_pose_wc);

        // set rendering state
        d_cam.Activate(*s_cam_);
        glClearColor(cs_.bg_.at(0), cs_.bg_.at(1), cs_.bg_.at(2), cs_.bg_.at(3));

        // draw horizontal grid
        draw_horizontal_grid();
        // draw the current camera frustum
        draw_current_cam_pose(gl_cam_pose_wc);
        // draw keyframes and graphs
        if (*menu_show_keyfrms_ || *menu_show_graph_ || *menu_show_essential_graph_) {
            std::vector<std::shared_ptr<stella_vslam::data::keyframe>> keyfrms;
            map_publisher_->get_keyframes(keyfrms);
            if (*menu_show_keyfrms_) {
                draw_keyframes(keyfrms);
            }
            if (*menu_show_graph_) {
                draw_covisibility_edges(keyfrms);
            }
            if (*menu_show_essential_graph_) {
                draw_spanning_tree_edges(keyfrms);
            }
        }
        // draw landmarks
        if (*menu_show_lms_) {
            std::vector<std::shared_ptr<stella_vslam::data::landmark>> landmarks;
            std::set<std::shared_ptr<stella_vslam::data::landmark>> local_landmarks;
            map_publisher_->get_landmarks(landmarks, local_landmarks);
            if (!landmarks.empty()) {
                draw_landmarks(landmarks, local_landmarks);
            }
        }

        pangolin::FinishFrame();

        // 2. draw the current frame image

        if (*menu_show_image_) {
            cv::imshow(frame_viewer_name_, frame_publisher_->draw_frame());
        }
        cv::waitKey(interval_ms_);

        // 3. state transition

        if (*menu_reset_) {
            reset();
        }

        check_state_transition();

        // 4. check termination flag

        if (*menu_terminate_ || pangolin::ShouldQuit()) {
            request_terminate();
        }

        if (terminate_is_requested()) {
            break;
        }
    }

    if (system_->tracker_is_paused()) {
        system_->resume_tracker();
    }

    system_->request_terminate();

    terminate();
}

void viewer::create_menu_panel() {
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(230));
    menu_follow_camera_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Follow Camera", true, true));
    menu_grid_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show Grid", false, true));
    menu_show_keyfrms_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show Keyframes", true, true));
    menu_show_lms_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show Landmarks", true, true));
    menu_show_local_map_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show Local Map", false, true));
    menu_show_graph_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show covisibility graph", true, true));
    menu_show_essential_graph_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show essential graph", true, true));
    menu_show_image_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Show image", true, true));
    menu_mapping_mode_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Mapping", mapping_mode_, true));
    menu_loop_detection_mode_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Loop Detection", loop_detection_mode_, true));
    menu_pause_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Pause", false, true));
    menu_reset_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Reset", false, false));
    menu_terminate_ = std::unique_ptr<pangolin::Var<bool>>(new pangolin::Var<bool>("menu.Terminate", false, false));
    menu_min_shared_lms_ = std::unique_ptr<pangolin::Var<int>>(new pangolin::Var<int>("menu.Min shared landmarks", 100, 1, 500));
    menu_kf_id_ = std::unique_ptr<pangolin::Var<std::string>>(new pangolin::Var<std::string>("menu.Keyframe ID", "0"));
    menu_frm_size_ = std::unique_ptr<pangolin::Var<float>>(new pangolin::Var<float>("menu.Frame Size", camera_size_, 0.01, 10.0, true));
    menu_keyfrm_size_ = std::unique_ptr<pangolin::Var<float>>(new pangolin::Var<float>("menu.KeyFrame Size", keyfrm_size_, 0.01, 10.0, true));
    menu_lm_size_ = std::unique_ptr<pangolin::Var<float>>(new pangolin::Var<float>("menu.Landmark Size", 1.0, 1e-1, 1e1, true));
}

void viewer::follow_camera(const pangolin::OpenGlMatrix& gl_cam_pose_wc) {
    if (*menu_follow_camera_ && follow_camera_) {
        s_cam_->Follow(gl_cam_pose_wc);
    }
    else if (*menu_follow_camera_ && !follow_camera_) {
        s_cam_->SetModelViewMatrix(pangolin::ModelViewLookAt(viewpoint_x_, viewpoint_y_, viewpoint_z_, 0, 0, 0, 0.0, -1.0, 0.0));
        s_cam_->Follow(gl_cam_pose_wc);
        follow_camera_ = true;
    }
    else if (!*menu_follow_camera_ && follow_camera_) {
        follow_camera_ = false;
    }
}

void viewer::draw_horizontal_grid() {
    if (!*menu_grid_) {
        return;
    }

    glLineWidth(1);
    glColor3fv(cs_.grid_.data());

    constexpr float interval_ratio = 0.1;
    pangolin::glDraw_y0(interval_ratio, 10);
}

pangolin::OpenGlMatrix viewer::get_current_cam_pose() {
    const auto cam_pose_cw = map_publisher_->get_current_cam_pose();
    const pangolin::OpenGlMatrix gl_cam_pose_wc(cam_pose_cw.inverse().eval());
    return gl_cam_pose_wc;
}

void viewer::draw_current_cam_pose(const pangolin::OpenGlMatrix& gl_cam_pose_wc) {
    // frustum size of the frame
    const float w = *menu_frm_size_;

    glLineWidth(camera_line_width_);
    glColor3fv(cs_.curr_cam_.data());
    draw_camera(gl_cam_pose_wc, w);
}

void viewer::draw_keyframes(std::vector<std::shared_ptr<stella_vslam::data::keyframe>>& keyfrms) {
    // frustum size of keyframes
    const float w = *menu_keyfrm_size_;

    int keyframe_id = parse_int(*menu_kf_id_);
    glLineWidth(keyfrm_line_width_);
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }
        if (keyframe_id != -1 && keyfrm->id_ == static_cast<unsigned int>(keyframe_id)) {
            glColor3fv(cs_.kf_line_selected_.data());
        }
        else {
            glColor3fv(cs_.kf_line_.data());
        }
        draw_camera(keyfrm->get_pose_wc(), w);
    }
}

void viewer::draw_covisibility_edges(std::vector<std::shared_ptr<stella_vslam::data::keyframe>>& keyfrms) {
    glLineWidth(graph_line_width_);

    glColor4fv(cs_.graph_line_.data());
    std::vector<Eigen::Vector3f> lines;
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }

        const stella_vslam::Vec3_t cam_center_1 = keyfrm->get_trans_wc();

        // covisibility graph
        const auto covisibilities = keyfrm->graph_node_->get_covisibilities_over_min_num_shared_lms(*menu_min_shared_lms_);
        if (!covisibilities.empty()) {
            for (const auto& covisibility : covisibilities) {
                if (!covisibility || covisibility->will_be_erased()) {
                    continue;
                }
                if (covisibility->id_ < keyfrm->id_) {
                    continue;
                }
                const stella_vslam::Vec3_t cam_center_2 = covisibility->get_trans_wc();
                lines.push_back(cam_center_1.cast<float>());
                lines.push_back(cam_center_2.cast<float>());
            }
        }
    }
    pangolin::glDrawLines(lines);
}

void viewer::draw_spanning_tree_edges(std::vector<std::shared_ptr<stella_vslam::data::keyframe>>& keyfrms) {
    glLineWidth(graph_line_width_);

    glColor4fv(cs_.graph_line_spanning_tree_.data());
    std::vector<Eigen::Vector3f> lines;
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }

        const stella_vslam::Vec3_t cam_center_1 = keyfrm->get_trans_wc();

        // spanning tree
        auto spanning_parent = keyfrm->graph_node_->get_spanning_parent();
        if (spanning_parent) {
            const stella_vslam::Vec3_t cam_center_2 = spanning_parent->get_trans_wc();
            lines.push_back(cam_center_1.cast<float>());
            lines.push_back(cam_center_2.cast<float>());
        }
    }
    pangolin::glDrawLines(lines);

    glColor4fv(cs_.graph_line_loop_edge_.data());
    lines.clear();
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }

        const stella_vslam::Vec3_t cam_center_1 = keyfrm->get_trans_wc();

        // loop edges
        const auto loop_edges = keyfrm->graph_node_->get_loop_edges();
        for (const auto& loop_edge : loop_edges) {
            if (!loop_edge) {
                continue;
            }
            if (loop_edge->id_ < keyfrm->id_) {
                continue;
            }
            const stella_vslam::Vec3_t cam_center_2 = loop_edge->get_trans_wc();
            lines.push_back(cam_center_1.cast<float>());
            lines.push_back(cam_center_2.cast<float>());
        }
    }
    pangolin::glDrawLines(lines);
}

void viewer::draw_landmarks(std::vector<std::shared_ptr<stella_vslam::data::landmark>>& landmarks,
                            std::set<std::shared_ptr<stella_vslam::data::landmark>>& local_landmarks) {
    glPointSize(point_size_ * *menu_lm_size_);
    glColor3fv(cs_.lm_.data());

    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> colors;
    for (const auto& lm : landmarks) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        if (*menu_show_local_map_ && local_landmarks.count(lm)) {
            continue;
        }
        const stella_vslam::Vec3_t pos_w = lm->get_pos_in_world();
        if (!*menu_show_local_map_) {
            const double score = lm->get_observed_ratio();
            const tinycolormap::Color score_color = tinycolormap::GetColor(score, tinycolormap::ColormapType::Turbo);
            const Eigen::Vector3f lm_color{static_cast<float>(score_color.r()), static_cast<float>(score_color.g()), static_cast<float>(score_color.b())};
            colors.push_back(lm_color);
        }
        points.push_back(pos_w.cast<float>());
    }
    if (*menu_show_local_map_) {
        pangolin::glDrawPoints(points);
    }
    else {
        pangolin::glDrawColoredVertices(points.size(), points.data(), colors.data(), GL_POINTS);
    }

    if (!*menu_show_local_map_) {
        return;
    }

    glPointSize(point_size_ * *menu_lm_size_);
    glColor3fv(cs_.local_lm_.data());
    points.clear();
    for (const auto& local_lm : local_landmarks) {
        if (local_lm->will_be_erased()) {
            continue;
        }
        const stella_vslam::Vec3_t pos_w = local_lm->get_pos_in_world();
        points.push_back(pos_w.cast<float>());
    }
    pangolin::glDrawPoints(points);
}

void viewer::draw_camera(const pangolin::OpenGlMatrix& gl_cam_pose_wc, const float width) const {
    glPushMatrix();
    gl_cam_pose_wc.Multiply();
    draw_frustum(width);
    glPopMatrix();
}

void viewer::draw_camera(const stella_vslam::Mat44_t& cam_pose_wc, const float width) const {
    glPushMatrix();
    glMultMatrixf(cam_pose_wc.transpose().cast<float>().eval().data());
    draw_frustum(width);
    glPopMatrix();
}

void viewer::draw_frustum(const float width) const {
    constexpr int image_width = 1080;
    constexpr int image_height = 720;
    constexpr float horizontal_fov = 2 * M_PI / 3; // 120 deg
    const float focal_length_pix = 0.5 * image_width / std::tan(0.5 * horizontal_fov);
    stella_vslam::Mat33_t Kinv = stella_vslam::Mat33_t::Identity();
    Kinv(0, 0) = 1.0 / focal_length_pix;
    Kinv(1, 1) = 1.0 / focal_length_pix;
    Kinv(0, 2) = -0.5 * image_width / focal_length_pix;
    Kinv(1, 2) = -0.5 * image_height / focal_length_pix;
    const float z = width / image_width * focal_length_pix;
    pangolin::glDrawFrustum(Kinv, image_width, image_height, z);
}

void viewer::reset() {
    // reset menu checks
    *menu_follow_camera_ = true;
    *menu_show_keyfrms_ = true;
    *menu_show_lms_ = true;
    *menu_show_local_map_ = true;
    *menu_show_graph_ = true;
    *menu_mapping_mode_ = mapping_mode_;
    *menu_loop_detection_mode_ = loop_detection_mode_;

    // reset menu button
    *menu_reset_ = false;
    *menu_terminate_ = false;

    // reset mapping mode
    if (mapping_mode_) {
        system_->enable_mapping_module();
    }
    else {
        system_->disable_mapping_module();
    }

    // reset loop detector
    if (loop_detection_mode_) {
        system_->enable_loop_detector();
    }
    else {
        system_->disable_loop_detector();
    }

    // reset internal state
    follow_camera_ = true;

    // execute reset
    system_->request_reset();
}

void viewer::check_state_transition() {
    // pause of tracker
    if (*menu_pause_ && !system_->tracker_is_paused()) {
        system_->pause_tracker();
    }
    else if (!*menu_pause_ && system_->tracker_is_paused()) {
        system_->resume_tracker();
    }

    // mapping module
    if (*menu_mapping_mode_ && !mapping_mode_) {
        system_->enable_mapping_module();
        mapping_mode_ = true;
    }
    else if (!*menu_mapping_mode_ && mapping_mode_) {
        system_->disable_mapping_module();
        mapping_mode_ = false;
    }

    // loop detector
    if (*menu_loop_detection_mode_ && !loop_detection_mode_) {
        system_->enable_loop_detector();
        loop_detection_mode_ = true;
    }
    else if (!*menu_loop_detection_mode_ && loop_detection_mode_) {
        system_->disable_loop_detector();
        loop_detection_mode_ = false;
    }
}

void viewer::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool viewer::is_terminated() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool viewer::terminate_is_requested() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void viewer::terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    is_terminated_ = true;
}

} // namespace pangolin_viewer
