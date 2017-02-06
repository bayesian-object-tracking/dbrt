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
 * \author C. Garcia Cifuentes (c.garciacifuentes@gmail.com)
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
 * \brief Represents the robot tracker publisher.
 */
template <typename State>
class RobotPublisher
{
public:
    RobotPublisher(
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
        const std::string& estimated_prefix,
        const std::string& connecting_frame_name);

    /**
     * \brief publish estimated tree on /tf topic
     */
    void publish_tf(const State& state, const ros::Time& time);

    /**
     * \brief publish estimated tree on /tf, connected to measured tree at the
     * root such that the connecting_frame_id in both trees align.
     */
    void publish_tf(const State& state,
                    const JointsObsrv& obsrv,
                    const ros::Time& time);

    /**
     * \brief publish estimated joint states to topic named
     * prefix_ + '/joint_states'
     */
    void publish_joint_state(const State& state, const ros::Time time);


protected:

    void publish_tf_tree(const State& state, const ros::Time& time);

    /**
     * \todo obsolete? -- better to not connect the trees than to connect them
     * with an arbitrary transform.
     */
    void publish_id_transform(const ros::Time& time,
                              const std::string& from,
                              const std::string& to);

    tf::StampedTransform get_root_transform(
        const std::map<std::string, double>& estimated_joint_positions,
        const std::map<std::string, double>& observed_joint_positions,
        const ros::Time& time);

    /**
     * This mapping function is required since the received JointsObsrv order
     * differs from the state type order provided through state.get_joint_map by
     * the URDF model.
     */
    void to_joint_map(const JointsObsrv& joint_values,
                      std::map<std::string, double>& named_joint_values) const;



protected:
    ros::NodeHandle node_handle_;

    // use this prefix to publish anything related to the estimated state
    std::string prefix_;

    // for publishing estimated robot state of /tf topic
    std::shared_ptr<robot_state_pub::RobotStatePublisher>
        robot_state_publisher_;

    // for publishing estimated joint states
    std::vector<std::string> joint_names_;
    sensor_msgs::JointState joint_state_msg_;
    ros::Publisher joint_state_publisher_;

    // These are needed for publishing a transform between the two roots
    // such that a connecting_frame_id is aligned in both.
    // \todo There is probably redundancy in all this publisher mess.
    std::string root_frame_name_;
    std::string connecting_frame_name_;
    RobotTransformer transformer_;

};
}
