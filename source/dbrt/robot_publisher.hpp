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
 * \file robot_publisher.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 * \author C. Garcia Cifuentes (c.garciacifuentes@gmail.com)
 */

#pragma once

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/distortion_models.h>

#include <fl/util/types.hpp>
#include <fl/util/profiling.hpp>

#include <dbot_ros/util/ros_interface.hpp>

#include <dbrt/robot_publisher.h>

namespace dbrt
{
class ObjectRenderTools
{
public:
protected:
};

template <typename State>
RobotPublisher<State>::RobotPublisher(
    const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
    const std::string& estimated_prefix,
    const std::string& connecting_frame_name)
    : node_handle_("~"),
      prefix_(estimated_prefix),
      robot_state_publisher_(
          std::make_shared<robot_state_pub::RobotStatePublisher>(
              urdf_kinematics->get_tree())),
      joint_names_(urdf_kinematics->get_joint_map()),
      root_frame_name_(urdf_kinematics->get_root_frame_id()),
      connecting_frame_name_(connecting_frame_name),
      transformer_(robot_state_publisher_)
{
    // setup basic joint angle message
    auto sz = joint_names_.size();
    joint_state_msg_.position.resize(sz);
    joint_state_msg_.effort.resize(sz);
    joint_state_msg_.velocity.resize(sz);
    joint_state_msg_.name.clear();
    for (int i = 0; i < sz; ++i) {
        joint_state_msg_.name.push_back(joint_names_[i]);
    }

    // joint angle publisher
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>(
        prefix_ + "/joint_states", 0);
}

template <typename State>
void RobotPublisher<State>::publish_joint_state(const State& state,
                                                const ros::Time time)
{
    ROS_FATAL_COND(joint_state_msg_.position.size() != state.size(),
                   "Joint state message and robot state sizes do not match");

    joint_state_msg_.header.stamp = time;
    for (int i = 0; i < state.size(); ++i)
    {
        joint_state_msg_.position[i] = state(i, 0);
    }

    joint_state_publisher_.publish(joint_state_msg_);
}

template <typename State>
void RobotPublisher<State>::publish_tf(const State& state,
                                       const ros::Time& time)
{
    /**
     * [Cris] I'm removing this id transform. Either we have and use an explicit
     * connecting_frame_id, or we don't connect at all.
     *
     * This may break some rviz visualization, but prevents the user from
     * inadvertedly looking up transforms that don't make sense.
     *
     * To obtain this id behavior, the user needs to set the
     * connecting_frame_id to be the root (in the corresponding config file).
     */
    // Publish id transform between roots
    //publish_id_transform(
    //    time,
    //    tf::resolve("", root_frame_name_),
    //    tf::resolve(prefix_, root_frame_name_));

    // Publish estimated tree
    publish_tf_tree(state, time);
}

template <typename State>
void RobotPublisher<State>::publish_tf(const State& state,
                                       const JointsObsrv& obsrv,
                                       const ros::Time& time)
{
    std::map<std::string, double> estimated_joint_positions;
    std::map<std::string, double> observed_joint_positions;
    state.GetJointState(estimated_joint_positions);
    to_joint_map(obsrv, observed_joint_positions);

    // Get the transform between the estimated root and measured root,
    // such that the estimated tree and measured tree are aligned
    // at the connecting frame
    tf::StampedTransform root_transform = get_root_transform(
        estimated_joint_positions, observed_joint_positions, time);

    // Publish transform between roots
    static tf::TransformBroadcaster br;
    br.sendTransform(root_transform);

    // Publish estimated tree
    publish_tf_tree(state, time);
}

template <typename State>
void RobotPublisher<State>::publish_tf_tree(const State& state,
                                            const ros::Time& time)
{
    // Publish movable joints
    std::map<std::string, double> joint_positions;
    state.GetJointState(joint_positions);
    robot_state_publisher_->publishTransforms(joint_positions, time, prefix_);

    // Publish fixed transforms
    robot_state_publisher_->publishFixedTransforms(prefix_, time);
}

template <typename State>
void RobotPublisher<State>::publish_id_transform(const ros::Time& time,
                                                 const std::string& from,
                                                 const std::string& to)
{
    if (from == to) return;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, time, from, to));
}

template <typename State>
tf::StampedTransform RobotPublisher<State>::get_root_transform(
    const std::map<std::string, double>& estimated_joint_positions,
    const std::map<std::string, double>& observed_joint_positions,
    const ros::Time& time)
{
    // Lookup transform target<-root in estimated tree
    transformer_.set_joints(estimated_joint_positions);
    tf::StampedTransform estimated_target_root;
    transformer_.lookup_transform(
        connecting_frame_name_, root_frame_name_, estimated_target_root);

    // Lookup transform root<-target in measured tree
    transformer_.set_joints(observed_joint_positions);
    tf::StampedTransform observed_root_target;
    transformer_.lookup_transform(
        root_frame_name_, connecting_frame_name_, observed_root_target);

    // Compose the two transforms
    tf::StampedTransform transform_root_root(
        observed_root_target * estimated_target_root,
        time,
        tf::resolve("", root_frame_name_),
        tf::resolve(prefix_, root_frame_name_));

    return transform_root_root;
}

template <typename State>
void RobotPublisher<State>::to_joint_map(
    const JointsObsrv& joint_values,
    std::map<std::string, double>& joint_map) const
{
    joint_map.clear();
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        joint_map[joint_names_[i]] = joint_values[i];
    }
}

}
