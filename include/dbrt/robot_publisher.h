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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <dbot/util/rigid_body_renderer.hpp>
#include <dbot/util/camera_data.hpp>
#include <dbot/util/object_model.hpp>

#include <dbot_ros/tracker_publisher.h>

#include <robot_state_pub/robot_state_publisher.h>
#include <dbrt/util/robot_transformer.hpp>
#include <dbrt/util/kinematics_from_urdf.hpp>

namespace dbrt
{

//\todo fix this quick hack
typedef Eigen::Matrix<fl::Real, Eigen::Dynamic, 1> JointsObsrv;


/**
 * \brief Represents the object tracker publisher. This publishes the object
 * estimated state and its marker.
 */
template <typename State>
class RobotTrackerPublisher: public dbot::TrackerPublisher<State>
{
public:
    RobotTrackerPublisher(
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
        const std::shared_ptr<dbot::RigidBodyRenderer>& renderer,
        const std::string& tf_prefix,
        const std::string& data_prefix,
        const std::string& target_frame_id,
        const std::string& measured_tf_prefix="");

    /// \todo THIS FUNCTION HAS TO GO AWAY!!
    void publish(const State& state,
                 const sensor_msgs::Image& obsrv_image,
                 const std::shared_ptr<dbot::CameraData>& camera_data);


    void publish_joint_state(const State& state, const ros::Time time);

    void publish_tf(const State& state, const ros::Time &time);

    void publish_tf_tree(const State& state, const ros::Time &time);

    void publish_root_link(const State& state, const ros::Time &time,
        const JointsObsrv& angle_measurements) ;


    void publish_image(const Eigen::VectorXd& depth_image,
                      const std::shared_ptr<dbot::CameraData>& camera_data,
                      const ros::Time& time);

    void publish_camera_info(const std::shared_ptr<dbot::CameraData>& camera_data,
                             const ros::Time &time);

    void publish_id_transform(const ros::Time& time, const std::string& from,
                              const std::string& to);

protected:



    // \todo Kind of duplicated wrt state.GetJointState().
    void get_joint_map_(const JointsObsrv& joint_values,
                        std::map<std::string, double>& named_joint_values) const;

    bool has_image_subscribers() const;

    void convert_to_depth_image_msg(
        const std::shared_ptr<dbot::CameraData>& camera_data,
        const Eigen::VectorXd& depth_image,
        sensor_msgs::Image& image);

    sensor_msgs::CameraInfoPtr create_camera_info(
            const std::shared_ptr<dbot::CameraData>& camera_data,
            const ros::Time &time);

    sensor_msgs::JointState joint_state_;
    ros::NodeHandle node_handle_;
    std::string tf_prefix_;
    std::shared_ptr<robot_state_pub::RobotStatePublisher> robot_state_publisher_;
    ros::Publisher pub_joint_state_;
    ros::Publisher pub_camera_info_;
    image_transport::Publisher pub_depth_image_;
    std::shared_ptr<dbot::RigidBodyRenderer> robot_renderer_;

    // These are needed for publishing a transform between the two roots
    // such that a target frame_id is aligned in both.
    // \todo There is probably redundancy in all this publisher mess.
    RobotTransformer transformer_;
    std::string target_frame_id_;
    std::string root_frame_id_;
    std::vector<std::string> joint_names_;
    std::string measured_tf_prefix_;
};

}
