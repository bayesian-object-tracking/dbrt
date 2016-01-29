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

#include <sensor_msgs/fill_image.h>

#include <fl/util/types.hpp>
#include <fl/util/profiling.hpp>

#include <dbot_ros/utils/ros_interface.hpp>

#include <dbrt/util/image_visualizer.hpp>
#include <dbrt/robot_tracker_publisher.h>

namespace dbrt
{
class ObjectRenderTools
{
public:
protected:
};

template <typename State>
RobotTrackerPublisher<State>::RobotTrackerPublisher(
    const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
    const std::shared_ptr<dbot::RigidBodyRenderer>& renderer,
    const std::string& tf_prefix)
    : node_handle_("~"), tf_prefix_(tf_prefix), robot_renderer_(renderer)
{
    pub_point_cloud_ = std::shared_ptr<ros::Publisher>(new ros::Publisher());

    *pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
        tf_prefix + "/XTION/depth/points", 0);

    boost::shared_ptr<image_transport::ImageTransport> it(
        new image_transport::ImageTransport(node_handle_));

    pub_depth_image_ = it->advertise(tf_prefix + "/XTION/depth/image", 0);
    pub_rgb_image_ = it->advertise(tf_prefix + "/XTION/depth/image_color", 0);

    // get the name of the root frame
    root_ = urdf_kinematics->GetRootFrameID();

    // initialize the robot state publisher
    robot_state_publisher_ =
        std::make_shared<robot_state_pub::RobotStatePublisher>(
            urdf_kinematics->GetTree());
}

template <typename State>
void RobotTrackerPublisher<State>::convert_to_rgb_depth_image_msg(
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const Eigen::VectorXd& depth_points,
    sensor_msgs::Image& image)
{
    vis::ImageVisualizer image_viz(camera_data->resolution().height,
                                   camera_data->resolution().width);

    image_viz.add_points(depth_points);
    image_viz.get_image(image);
}

template <typename State>
void RobotTrackerPublisher<State>::convert_to_depth_image_msg(
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const Eigen::VectorXd& depth_image,
    sensor_msgs::Image& image)
{
    Eigen::VectorXf float_vector = depth_image.cast<float>();

    sensor_msgs::fillImage(image,
                           sensor_msgs::image_encodings::TYPE_32FC1,
                           camera_data->resolution().height,
                           camera_data->resolution().width,
                           camera_data->resolution().width * sizeof(float),
                           float_vector.data());
}

template <typename State>
bool RobotTrackerPublisher<State>::has_image_subscribers() const
{
    return pub_rgb_image_.getNumSubscribers() > 0 ||
           pub_depth_image_.getNumSubscribers() > 0 ||
           pub_point_cloud_->getNumSubscribers() > 0;
}

template <typename State>
bool RobotTrackerPublisher<State>::has_point_cloud_subscribers() const
{
    return pub_point_cloud_->getNumSubscribers() > 0;
}

template <typename State>
void RobotTrackerPublisher<State>::publish(
    State& state,
    const sensor_msgs::Image& image,
    const std::shared_ptr<dbot::CameraData>& camera_data)
{
    ros::Time t = ros::Time::now();

    // make sure there is a identity transformation between base of real
    // robot and estimated robot
    publishTransform(t, root_, tf::resolve(tf_prefix_, root_));

    // publish movable joints
    std::map<std::string, double> joint_positions;
    state.GetJointState(joint_positions);
    robot_state_publisher_->publishTransforms(joint_positions, t, tf_prefix_);

    // publish fixed transforms
    robot_state_publisher_->publishFixedTransforms(tf_prefix_, t);

    if (!has_image_subscribers() && !has_point_cloud_subscribers()) return;

    Eigen::VectorXd depth_image;
    robot_renderer_->Render(
        state, depth_image, std::numeric_limits<double>::quiet_NaN());

    publishImage(depth_image, camera_data, t, tf_prefix_);
    publishPointCloud(depth_image, camera_data, t, tf_prefix_);
}

template <typename State>
void RobotTrackerPublisher<State>::publishImage(
    const Eigen::VectorXd& depth_image,
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const ros::Time& time,
    const std::string& tf_prefix)
{

    if (pub_rgb_image_.getNumSubscribers() > 0)
    {
        sensor_msgs::Image ros_image;
        convert_to_rgb_depth_image_msg(camera_data, depth_image, ros_image);

        ros_image.header.frame_id = tf_prefix + camera_data->frame_id();
        ros_image.header.stamp = time;
        pub_rgb_image_.publish(ros_image);
    }

    if (pub_depth_image_.getNumSubscribers() > 0)
    {
        sensor_msgs::Image ros_image;
        convert_to_depth_image_msg(camera_data, depth_image, ros_image);

        ros_image.header.frame_id = tf_prefix + camera_data->frame_id();
        ros_image.header.stamp = time;
        pub_depth_image_.publish(ros_image);
    }
}

template <typename State>
void RobotTrackerPublisher<State>::publishTransform(const ros::Time& time,
                                                    const std::string& from,
                                                    const std::string& to)
{
    if (from.compare(to) == 0) return;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, time, from, to));
}

template <typename State>
void RobotTrackerPublisher<State>::publishPointCloud(
    const Eigen::VectorXd& depth_image,
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const ros::Time& time,
    const std::string& tf_prefix)
{
    Eigen::MatrixXd camera_matrix = camera_data->camera_matrix();
    camera_matrix.topLeftCorner(2, 3) *=
        double(camera_data->downsampling_factor());

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Ptr points =
        boost::make_shared<sensor_msgs::PointCloud2>();

    const int nRows = camera_data->resolution().height;
    const int nCols = camera_data->resolution().width;

    //    points->header.frame_id = tf::resolve(tf_prefix,
    //    camera_data->frame_id());
    points->header.frame_id = tf_prefix + camera_data->frame_id();
    points->header.stamp = time;
    points->width = nCols;
    points->height = nRows;
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

    for (size_t v = 0; v < nRows; ++v)
    {
        for (size_t u = 0; u < nCols; ++u)
        {
            float depth = depth_image(v * nCols + u, 0);
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
                int ux = u * camera_data->downsampling_factor();
                int vx = v * camera_data->downsampling_factor();
                // depth is valid
                float x = ((float)ux - camera_matrix(0, 2)) * depth /
                          camera_matrix(0, 0);
                float y = ((float)vx - camera_matrix(1, 2)) * depth /
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
