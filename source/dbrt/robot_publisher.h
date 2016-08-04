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
 * \file robot_publisher.h
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <dbot/rigid_body_renderer.hpp>
#include <dbot/camera_data.hpp>
#include <dbot/object_model.hpp>

#include <dbrt/robot_transformer.h>
#include <dbrt/kinematics_from_urdf.h>

#include <robot_state_pub/robot_state_publisher.h>

namespace dbrt
{
//\todo fix this quick hack
typedef Eigen::Matrix<fl::Real, Eigen::Dynamic, 1> JointsObsrv;

/**
 * \brief Represents the object tracker publisher. This publishes the object
 * estimated state and its marker.
 */
template <typename State>
class RobotPublisher
{
public:
    RobotPublisher(
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
        const std::string& prefix,
        const std::string& target_frame_id);

    void publish_tf(const State& state, const ros::Time& time);
    void publish_tf(const State& state,
                    const JointsObsrv& obsrv,
                    const ros::Time& time);

    void publish_joint_state(const State& state, const ros::Time time);

protected:
    /**
     * This mapping function is required since the received JointsObsrv order
     * differs from the state type order provided through state.get_joint_map by
     * the URDF model.
     */
    void to_joint_map(const JointsObsrv& joint_values,
                      std::map<std::string, double>& named_joint_values) const;
    void publish_tf_tree(const State& state, const ros::Time& time);
    void publish_id_transform(const ros::Time& time,
                              const std::string& from,
                              const std::string& to);
    tf::StampedTransform get_root_transform(
        const std::map<std::string, double>& state_joint_map,
        const std::map<std::string, double>& obsrv_joint_map,
        const ros::Time& time);


protected:
    sensor_msgs::JointState joint_state_;
    ros::NodeHandle node_handle_;
    std::string prefix_;
    std::shared_ptr<robot_state_pub::RobotStatePublisher>
        robot_state_publisher_;
    ros::Publisher pub_joint_state_;

    // These are needed for publishing a transform between the two roots
    // such that a target frame_id is aligned in both.
    // \todo There is probably redundancy in all this publisher mess.
    RobotTransformer transformer_;
    std::string connecting_frame_id;
    std::string root_frame_id_;
    std::vector<std::string> joint_names_;

};
}
