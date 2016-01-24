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

/**
 * \file robot_tracker_publisher.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <fl/util/types.hpp>
#include <fl/util/profiling.hpp>

#include <dbot_ros/utils/ros_interface.hpp>

#include <dbrt/util/image_visualizer.hpp>
#include <dbrt/robot_tracker_publisher.h>

namespace dbrt
{
template <typename Tracker>
RobotTrackerPublisher<Tracker>::RobotTrackerPublisher(
    const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
    const std::shared_ptr<dbot::RigidBodyRenderer>& renderer)
    : node_handle_("~"), tf_prefix_("MEAN"), robot_renderer_(renderer)
{
    pub_point_cloud_ = std::shared_ptr<ros::Publisher>(new ros::Publisher());

    *pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
        "/XTION/depth/points", 5);

    boost::shared_ptr<image_transport::ImageTransport> it(
        new image_transport::ImageTransport(node_handle_));
    pub_rgb_image_ = it->advertise("/XTION/depth/image_color", 5);

    // get the name of the root frame
    root_ = urdf_kinematics->GetRootFrameID();

    // initialize the robot state publisher
    robot_state_publisher_ =
        std::make_shared<robot_state_pub::RobotStatePublisher>(
            urdf_kinematics->GetTree());
}

template <typename Tracker>
void RobotTrackerPublisher<Tracker>::publish(
    State& state,
    const sensor_msgs::Image& image,
    const std::shared_ptr<dbot::CameraData>& camera_data)
{
    // DEBUG to see depth images
    std::vector<Eigen::Matrix3d> rotations(state.count_parts());
    std::vector<Eigen::Vector3d> translations(state.count_parts());
    for (size_t i = 0; i < rotations.size(); i++)
    {
        rotations[i] = state.component(i).orientation().rotation_matrix();
        translations[i] = state.component(i).position();
    }


    auto org_image = ri::Ros2Eigen<typename fl::Real>(
        image, camera_data->downsampling_factor());
    robot_renderer_->set_poses(rotations, translations);
    std::vector<int> indices;
    std::vector<float> depth;
    robot_renderer_->Render(camera_data->camera_matrix(),
                            org_image.rows(),
                            org_image.cols(),
                            indices,
                            depth);
    vis::ImageVisualizer image_viz(org_image.rows(), org_image.cols());
    image_viz.set_image(org_image);
    image_viz.add_points(indices, depth);

    std::map<std::string, double> joint_positions;
    state.GetJointState(joint_positions);

    ros::Time t = ros::Time::now();
    // publish movable joints
    robot_state_publisher_->publishTransforms(joint_positions, t, tf_prefix_);
    // make sure there is a identity transformation between base of real
    // robot and estimated robot
    publishTransform(t, root_, tf::resolve(tf_prefix_, root_));
    // publish fixed transforms
    robot_state_publisher_->publishFixedTransforms(tf_prefix_);
    // publish image
    sensor_msgs::Image overlay;
    image_viz.get_image(overlay);
    publishImage(t, camera_data->frame_id(), overlay);

    // publish point cloud
    Eigen::MatrixXd full_image = ri::Ros2Eigen<fl::Real>(image, 1);
    Eigen::MatrixXd temp_camera_matrix = camera_data->camera_matrix();
    temp_camera_matrix.topLeftCorner(2, 3) *=
        double(camera_data->downsampling_factor());
    publishPointCloud(
        full_image, t, camera_data->frame_id(), temp_camera_matrix);
}

template <typename Tracker>
void RobotTrackerPublisher<Tracker>::publishImage(const ros::Time& time,
                                                  const std::string& frame_id,
                                                  sensor_msgs::Image& image)
{
    image.header.frame_id = tf::resolve(tf_prefix_, frame_id);
    image.header.stamp = time;
    pub_rgb_image_.publish(image);
}

template <typename Tracker>
void RobotTrackerPublisher<Tracker>::publishTransform(const ros::Time& time,
                                                      const std::string& from,
                                                      const std::string& to)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, time, from, to));
}

template <typename Tracker>
void RobotTrackerPublisher<Tracker>::publishPointCloud(
    const Eigen::MatrixXd& image,
    const ros::Time& stamp,
    const std::string& frame_id,
    const Eigen::MatrixXd& camera_matrix)
{
    if (pub_point_cloud_->getNumSubscribers() == 0)
    {
        return;
    }

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Ptr points =
        boost::make_shared<sensor_msgs::PointCloud2>();

    points->header.frame_id = tf::resolve(tf_prefix_, frame_id);
    points->header.stamp = stamp;
    points->width = image.cols();
    points->height = image.rows();
    points->is_dense = false;
    points->is_bigendian = false;
    points->fields.resize(3);
    points->fields[0].name = "x";
    points->fields[1].name = "y";
    points->fields[2].name = "z";
    int offset = 0;
    for (size_t d = 0; d < points->fields.size(); ++d, offset += sizeof(float))
    {
        points->fields[d].offset = offset;
        points->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        points->fields[d].count = 1;
    }

    points->point_step = offset;
    points->row_step = points->point_step * points->width;

    points->data.resize(points->width * points->height * points->point_step);

    for (size_t u = 0, nRows = image.rows(), nCols = image.cols(); u < nCols;
         ++u)
    {
        for (size_t v = 0; v < nRows; ++v)
        {
            float depth = image(v, u);
            if (depth != depth)  // || depth==0.0)
            {
                // depth is invalid
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[0].offset],
                       &bad_point,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[1].offset],
                       &bad_point,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[2].offset],
                       &bad_point,
                       sizeof(float));
            }
            else
            {
                // depth is valid
                float x = ((float)u - camera_matrix(0, 2)) * depth /
                          camera_matrix(0, 0);
                float y = ((float)v - camera_matrix(1, 2)) * depth /
                          camera_matrix(1, 1);
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[0].offset],
                       &x,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[1].offset],
                       &y,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[2].offset],
                       &depth,
                       sizeof(float));
            }
        }
    }

    pub_point_cloud_->publish(points);
}
}
