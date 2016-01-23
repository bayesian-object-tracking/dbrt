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
 * \file robot_tracker_publisher.h
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <dbot/util/rigid_body_renderer.hpp>
#include <dbot/util/camera_data.hpp>
#include <dbot/util/object_model.hpp>

#include <dbot_ros/tracker_publisher.h>

#include <robot_state_pub/robot_state_publisher.h>

#include <brt/utils/kinematics_from_urdf.hpp>

namespace brt
{
/**
 * \brief Represents the object tracker publisher. This publishes the object
 * estimated state and its marker.
 */
template <typename Tracker>
class RobotTrackerPublisher : public dbot::TrackerPublisher<Tracker>
{
public:
    typedef typename Tracker::State State;

public:
    RobotTrackerPublisher(
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics);

    void publish(const State& state,
                 const sensor_msgs::Image& image,
                 const std::shared_ptr<dbot::CameraData>& camera_data);

    void publishImage(const ros::Time& time,
                      const std::string& frame_id,
                      sensor_msgs::Image& image);

    void publishTransform(const ros::Time& time,
                          const std::string& from,
                          const std::string& to);

    void publishPointCloud(const Eigen::MatrixXd& image,
                           const ros::Time& stamp,
                           const std::string& frame_id,
                           const Eigen::MatrixXd& camera_matrix);

protected:
    ros::NodeHandle node_handle_;
    std::string tf_prefix_;
    std::string root_;
    std::shared_ptr<robot_state_pub::RobotStatePublisher>
        robot_state_publisher_;
    std::shared_ptr<ros::Publisher> pub_point_cloud_;
    image_transport::Publisher pub_rgb_image_;
    std::shared_ptr<dbot::RigidBodyRenderer> robot_renderer_;
};
}
