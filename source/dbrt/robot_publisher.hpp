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
RobotTrackerPublisher<State>::RobotTrackerPublisher(
    const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
    const std::string& tf_prefix,
    const std::string& data_prefix,
    const std::string& target_frame_id)
    : node_handle_("~"),
      tf_prefix_(tf_prefix),
      target_frame_id_(target_frame_id),
      robot_state_publisher_(
          std::make_shared<robot_state_pub::RobotStatePublisher>(
              urdf_kinematics->GetTree())),
      transformer_(robot_state_publisher_)
{
    pub_joint_state_ = node_handle_.advertise<sensor_msgs::JointState>(
        data_prefix + "/joint_states", 0);

    // get the name of the root frame
    root_frame_id_ = urdf_kinematics->GetRootFrameID();

    // get joint map
    joint_names_ = urdf_kinematics->GetJointMap();

    // setup basic joint state message
    joint_state_.position.resize(joint_names_.size());
    joint_state_.effort.resize(joint_names_.size());
    joint_state_.velocity.resize(joint_names_.size());
    for (int i = 0; i < joint_names_.size(); ++i)
    {
        joint_state_.name.push_back(joint_names_[i]);
    }

    pub_camera_info_ = node_handle_.advertise<sensor_msgs::CameraInfo>(
        data_prefix + "/XTION/depth/camera_info", 0);

    boost::shared_ptr<image_transport::ImageTransport> it(
        new image_transport::ImageTransport(node_handle_));

    pub_depth_image_ = it->advertise(data_prefix + "/XTION/depth/image", 0);
}

template <typename State>
void RobotTrackerPublisher<State>::publish_joint_state(const State& state,
                                                       const ros::Time time)
{
    ROS_FATAL_COND(joint_state_.position.size() != state.size(),
                   "Joint state message and robot state sizes do not match");

    joint_state_.header.stamp = time;
    for (int i = 0; i < state.size(); ++i)
    {
        joint_state_.position[i] = state(i, 0);
    }

    pub_joint_state_.publish(joint_state_);
}

template <typename State>
void RobotTrackerPublisher<State>::to_joint_map(
    const JointsObsrv& joint_values,
    std::map<std::string, double>& joint_map) const
{
    joint_map.clear();
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        joint_map[joint_names_[i]] = joint_values[i];
    }
}

template <typename State>
void RobotTrackerPublisher<State>::publish_tf(const State& state,
                                              const ros::Time& time)
{
    publish_id_transform(
        time, root_frame_id_, tf::resolve(tf_prefix_, root_frame_id_));
    publish_tf_tree(state, time);
}

template <typename State>
tf::StampedTransform RobotTrackerPublisher<State>::get_root_transform(
    const std::map<std::string, double>& joint_map_1,
    const std::map<std::string, double>& joint_map_2,
    const std::string& connecting_frame,
    const ros::Time& time)
{
    // Lookup transform from target to root of joint map 1
    transformer_.set_joints(joint_map_1);
    tf::StampedTransform transform_t_r;
    transformer_.lookup_transform(
        target_frame_id_, root_frame_id_, transform_t_r);

    // Lookup transform from root to target of joint map 2
    transformer_.set_joints(joint_map_2);
    tf::StampedTransform transform_r_t;
    transformer_.lookup_transform(
        root_frame_id_, target_frame_id_, transform_r_t);

    // Compose the two tf_transforms
    tf::StampedTransform transform_r_r(transform_r_t * transform_t_r,
                                       time,
                                       root_frame_id_,
                                       tf::resolve(tf_prefix_, root_frame_id_));

    return transform_r_r;
}

template <typename State>
void RobotTrackerPublisher<State>::publish_tf(const State& state,
                                              const JointsObsrv& joints_obsrv,
                                              const ros::Time& time)
{
    std::map<std::string, double> state_joint_map;
    std::map<std::string, double> obsrv_joint_map;
    state.GetJointState(state_joint_map);
    to_joint_map(joints_obsrv, obsrv_joint_map);

    // get the transform between the estimated root and measured root both
    // linked at the specified target frame
    tf::StampedTransform root_transform = get_root_transform(
        state_joint_map, obsrv_joint_map, target_frame_id_, time);

    static tf::TransformBroadcaster br;
    br.sendTransform(root_transform);

    publish_tf_tree(state, time);
}

template <typename State>
void RobotTrackerPublisher<State>::publish_tf_tree(const State& state,
                                                   const ros::Time& time)
{
    // publish movable joints
    std::map<std::string, double> joint_positions;
    state.GetJointState(joint_positions);
    robot_state_publisher_->publishTransforms(
        joint_positions, time, tf_prefix_);

    // publish fixed transforms
    robot_state_publisher_->publishFixedTransforms(tf_prefix_, time);
}

template <typename State>
void RobotTrackerPublisher<State>::publish_id_transform(const ros::Time& time,
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
    return pub_depth_image_.getNumSubscribers() > 0;
}

template <typename State>
void RobotTrackerPublisher<State>::publish_image(
    const Eigen::VectorXd& depth_image,
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const ros::Time& time)
{
    if (pub_depth_image_.getNumSubscribers() > 0)
    {
        sensor_msgs::Image ros_image;
        convert_to_depth_image_msg(camera_data, depth_image, ros_image);

        ros_image.header.frame_id = tf_prefix_ + camera_data->frame_id();
        ros_image.header.stamp = time;
        pub_depth_image_.publish(ros_image);
    }
}

template <typename State>
void RobotTrackerPublisher<State>::publish_camera_info(
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const ros::Time& time)
{
    auto camera_info = create_camera_info(camera_data, time);
    pub_camera_info_.publish(camera_info);
}

template <typename State>
sensor_msgs::CameraInfoPtr RobotTrackerPublisher<State>::create_camera_info(
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const ros::Time& time)
{
    sensor_msgs::CameraInfoPtr info_msg =
        boost::make_shared<sensor_msgs::CameraInfo>();

    info_msg->header.stamp = time;
    // if internal registration is used, rgb camera intrinsic parameters are
    // used
    info_msg->header.frame_id = camera_data->frame_id();
    info_msg->width = camera_data->native_resolution().width;
    info_msg->height = camera_data->native_resolution().height;

#if ROS_VERSION_MINIMUM(1, 3, 0)
    info_msg->D = std::vector<double>(5, 0.0);
    info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
    info_msg->D.assign(0.0);
#endif
    info_msg->K.assign(0.0);
    info_msg->R.assign(0.0);
    info_msg->P.assign(0.0);

    auto camera_matrix = camera_data->camera_matrix();
    camera_matrix.topLeftCorner(2, 3) *=
        double(camera_data->downsampling_factor());

    for (size_t col = 0; col < 3; col++)
        for (size_t row = 0; row < 3; row++)
            info_msg->K[col + row * 3] = camera_matrix(row, col);

    // no rotation: identity
    info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
    // no rotation, no translation => P=K(I|0)=(K|0)
    info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
    info_msg->P[2] = info_msg->K[2];
    info_msg->P[6] = info_msg->K[5];
    info_msg->P[10] = 1.0;

    return info_msg;
}
}
