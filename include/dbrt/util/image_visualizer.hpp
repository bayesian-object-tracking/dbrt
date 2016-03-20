/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

#ifndef POSE_TRACKING_INTERFACE_UTILS_IMAGE_VISUALIZER_HPP
#define POSE_TRACKING_INTERFACE_UTILS_IMAGE_VISUALIZER_HPP

#include <vector>
#include <string>
#include "Eigen/Core"
// cv.h cannot be included in image_visualizer.hpp and therefore cv::Mat has to
// be
// avoided as input to any function in image_visualizer
//#include <cv.h>

#include <sensor_msgs/Image.h>

namespace vis
{
class ImageVisualizer
{
public:
    ImageVisualizer(const int& n_rows, const int& n_cols);

    ~ImageVisualizer();

    void set_image(const Eigen::MatrixXd& image,
                   const float& min_value = 0,
                   const float& max_value = 0,
                   const bool& invert_image = false);

    void set_image(const std::vector<float>& image,
                   const float& min_value = 0,
                   const float& max_value = 0,
                   const bool& invert_image = false);

    void add_points(const std::vector<Eigen::Vector3f>& points,
                    const Eigen::Matrix3f& camera_matrix,
                    const Eigen::Matrix3f& R = Eigen::Matrix3f::Identity(),
                    const Eigen::Vector3f& t = Eigen::Vector3f::Zero(),
                    const std::vector<float>& colors = std::vector<float>(0));

    void add_points(const Eigen::Matrix<Eigen::Vector3d, -1, -1>& points,
                    const Eigen::Matrix3d& camera_matrix,
                    const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(),
                    const Eigen::Vector3d& t = Eigen::Vector3d::Zero(),
                    const std::vector<float>& colors = std::vector<float>(0));

    void add_points(const Eigen::VectorXd &depth_image);

    void add_points(const std::vector<int>& point_indices,
                    const std::vector<float>& colors = std::vector<float>(0));

    char show_image(const std::string& window_name = "dini mer",
                    const int& window_width = 500,
                    const int& window_height = 500,
                    const int& delay = 0) const;

    // void get_image(Eigen::MatrixXd &image) const;
    void get_image(sensor_msgs::Image& image) const;

private:
    void* image_;
    const int n_rows_, n_cols_;

    void Cart2Index(const Eigen::Vector3f& cart,
                    const Eigen::Matrix3f& camera_matrix,
                    int& row,
                    int& col) const;
};
}

#endif
