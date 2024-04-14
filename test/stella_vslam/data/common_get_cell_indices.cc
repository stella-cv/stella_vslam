#include "stella_vslam/data/common.h"
#include "stella_vslam/camera/perspective.h"

#include <gtest/gtest.h>

using namespace stella_vslam;

camera::perspective create_perspective_camera(const unsigned int cols, const unsigned int rows,
                                              const float k1 = 0.0, const float k2 = 0.0) {
    using namespace camera;
    return perspective("perspective", setup_type_t::Monocular, color_order_t::RGB,
                       cols, rows, 30.0, static_cast<double>(rows), static_cast<double>(rows),
                       cols / 2.0, rows / 2.0, k1, k2, 0.0, 0.0, 0.0);
}

TEST(common, valid_cases_1) {
    // create an example perspective camera
    constexpr unsigned int cols = 2000;
    constexpr unsigned int rows = 1000;
    auto cam = create_perspective_camera(cols, rows, -0.1, 0.1);
    constexpr unsigned int num_grid_cols = 64;
    constexpr unsigned int num_grid_rows = 48;
    // create keypoints and those grid IDs
    std::vector<std::pair<cv::KeyPoint, std::pair<int, int>>> test_cases;
    constexpr float eps = 0.01;
    // - corners
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.min_x_, cam.img_bounds_.min_y_, 0.0},
                                           std::make_pair(0, 0))); // top left
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.max_x_ - eps, cam.img_bounds_.min_y_, 0.0},
                                           std::make_pair(num_grid_cols - 1, 0))); // top right
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.min_x_, cam.img_bounds_.max_y_ - eps, 0.0},
                                           std::make_pair(0, num_grid_rows - 1))); // bottop left
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.max_x_ - eps, cam.img_bounds_.max_y_ - eps, 0.0},
                                           std::make_pair(num_grid_cols - 1, num_grid_rows - 1))); // bottom right
    // - edges
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cols / 2.0, cam.img_bounds_.min_y_, 0.0},
                                           std::make_pair(num_grid_cols / 2 - 1, 0))); // top center
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cols / 2.0, cam.img_bounds_.max_y_ - eps, 0.0},
                                           std::make_pair(num_grid_cols / 2 - 1, num_grid_rows - 1))); // bottom center
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.min_x_, rows / 2.0, 0.0},
                                           std::make_pair(0, num_grid_rows / 2 - 1))); // left center
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.max_x_ - eps, rows / 2.0, 0.0},
                                           std::make_pair(num_grid_cols - 1, num_grid_rows / 2 - 1))); // right center

    // check
    double inv_cell_width = static_cast<double>(num_grid_cols) / (cam.img_bounds_.max_x_ - cam.img_bounds_.min_x_);
    double inv_cell_height = static_cast<double>(num_grid_rows) / (cam.img_bounds_.max_y_ - cam.img_bounds_.min_y_);
    for (const auto& test_case : test_cases) {
        // extract information
        const auto& keypt = test_case.first;
        const auto idx_x = test_case.second.first;
        const auto idx_y = test_case.second.second;
        // run the function
        int est_idx_x = -1;
        int est_idx_y = -1;
        const auto found = data::get_cell_indices(&cam, keypt, num_grid_cols, num_grid_rows, inv_cell_width, inv_cell_height, est_idx_x, est_idx_y);
        EXPECT_TRUE(found);
        EXPECT_EQ(est_idx_x, idx_x);
        EXPECT_EQ(est_idx_y, idx_y);
    }
}

TEST(common, valid_cases_2) {
    // create an example perspective camera
    constexpr unsigned int cols = 2000;
    constexpr unsigned int rows = 1000;
    auto cam = create_perspective_camera(cols, rows);
    constexpr unsigned int num_grid_cols = 64;
    constexpr unsigned int num_grid_rows = 48;
    double inv_cell_width = static_cast<double>(num_grid_cols) / (cam.img_bounds_.max_x_ - cam.img_bounds_.min_x_);
    double inv_cell_height = static_cast<double>(num_grid_rows) / (cam.img_bounds_.max_y_ - cam.img_bounds_.min_y_);
    // create keypoints and those grid IDs
    std::vector<std::pair<cv::KeyPoint, std::pair<int, int>>> test_cases;
    constexpr float eps = 0.01;
    // - corners of each cell
    for (unsigned int idx_x = 0; idx_x < num_grid_cols; ++idx_x) {
        for (unsigned int idx_y = 0; idx_y < num_grid_rows; ++idx_y) {
            // top left
            {
                const float x = idx_x * (1.0 / inv_cell_width) + eps;
                const float y = idx_y * (1.0 / inv_cell_height) + eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
            // top right
            {
                const float x = (idx_x + 1) * (1.0 / inv_cell_width) - eps;
                const float y = idx_y * (1.0 / inv_cell_height) + eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
            // bottom left
            {
                const float x = idx_x * (1.0 / inv_cell_width) + eps;
                const float y = (idx_y + 1) * (1.0 / inv_cell_height) - eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
            // bottom right
            {
                const float x = (idx_x + 1) * (1.0 / inv_cell_width) - eps;
                const float y = (idx_y + 1) * (1.0 / inv_cell_height) - eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
        }
    }

    // check
    for (const auto& test_case : test_cases) {
        // extract information
        const auto& keypt = test_case.first;
        const auto idx_x = test_case.second.first;
        const auto idx_y = test_case.second.second;
        // run the function
        int est_idx_x = -1;
        int est_idx_y = -1;
        const auto found = data::get_cell_indices(&cam, keypt, num_grid_cols, num_grid_rows, inv_cell_width, inv_cell_height, est_idx_x, est_idx_y);
        EXPECT_TRUE(found);
        EXPECT_EQ(est_idx_x, idx_x);
        EXPECT_EQ(est_idx_y, idx_y);
    }
}

TEST(common, invalid_cases) {
    // create an example perspective camera
    constexpr unsigned int cols = 2000;
    constexpr unsigned int rows = 1000;
    auto cam = create_perspective_camera(cols, rows, -0.1, 0.1);
    constexpr unsigned int num_grid_cols = 64;
    constexpr unsigned int num_grid_rows = 48;
    // create keypoints
    std::vector<cv::KeyPoint> test_cases;
    constexpr float eps = 0.01;
    // clang-format off
    // - invalid corners
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.min_x_ - eps, cam.img_bounds_.min_y_ - eps, 0.0}); // top left
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.max_x_, cam.img_bounds_.min_y_ - eps, 0.0}); // top right
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.min_x_ - eps, cam.img_bounds_.max_y_, 0.0}); // bottom left
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.max_x_, cam.img_bounds_.max_y_, 0.0}); // bottom right
    // - invalid edges
    test_cases.emplace_back(cv::KeyPoint{cols / 2.0, cam.img_bounds_.min_y_ - eps, 0.0}); // top center
    test_cases.emplace_back(cv::KeyPoint{cols / 2.0, cam.img_bounds_.max_y_, 0.0}); // bottom center
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.min_x_ - eps, rows / 2.0, 0.0}); // left center
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.max_x_, rows / 2.0, 0.0}); // right center
    // clang-format on

    // check
    double inv_cell_width = static_cast<double>(num_grid_cols) / (cam.img_bounds_.max_x_ - cam.img_bounds_.min_x_);
    double inv_cell_height = static_cast<double>(num_grid_rows) / (cam.img_bounds_.max_y_ - cam.img_bounds_.min_y_);
    for (const auto& test_case : test_cases) {
        // extract information
        const auto& keypt = test_case;
        // run the function
        int est_idx_x = -1;
        int est_idx_y = -1;
        const auto found = data::get_cell_indices(&cam, keypt, num_grid_cols, num_grid_rows, inv_cell_width, inv_cell_height, est_idx_x, est_idx_y);
        EXPECT_FALSE(found);
    }
}
