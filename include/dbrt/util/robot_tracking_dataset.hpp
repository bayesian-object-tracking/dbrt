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

#ifndef POSE_TRACKING_INTERFACE_UTILS_ROBOT_TRACKING_DATASET_HPP
#define POSE_TRACKING_INTERFACE_UTILS_ROBOT_TRACKING_DATASET_HPP

#include <dbot_ros/utils/tracking_dataset.h>

class RobotTrackingDataset :  public TrackingDataset
{
public:

  RobotTrackingDataset(const std::string& path);

  void AddFrame(const sensor_msgs::Image::ConstPtr& image,
        const sensor_msgs::CameraInfo::ConstPtr& info,
        const sensor_msgs::JointState::ConstPtr& ground_truth_joints,
        const sensor_msgs::JointState::ConstPtr& noisy_joints,
        const tf::tfMessage::ConstPtr& tf,
        const tf::tfMessage::ConstPtr& fixed_tf,
        const Eigen::VectorXd& ground_truth = Eigen::VectorXd(),
        const Eigen::VectorXd& deviation = Eigen::VectorXd());

  void AddFrame(const sensor_msgs::Image::ConstPtr& image,
        const sensor_msgs::CameraInfo::ConstPtr& info,
        const sensor_msgs::JointState::ConstPtr& ground_truth_joints,
        const sensor_msgs::JointState::ConstPtr& noisy_joints,
        const tf::tfMessage::ConstPtr& tf,
        const tf::tfMessage::ConstPtr& fixed_tf);

  void AddFrame(const sensor_msgs::Image::ConstPtr& image,
        const sensor_msgs::CameraInfo::ConstPtr& info,
        const sensor_msgs::JointState::ConstPtr& ground_truth_joints,
        const sensor_msgs::JointState::ConstPtr& noisy_joints,
        const Eigen::VectorXd& ground_truth = Eigen::VectorXd(),
        const Eigen::VectorXd& deviation = Eigen::VectorXd());

  void AddFrame(const sensor_msgs::Image::ConstPtr& image,
        const sensor_msgs::CameraInfo::ConstPtr& info,
        const sensor_msgs::JointState::ConstPtr& ground_truth_joints,
        const sensor_msgs::JointState::ConstPtr& noisy_joints);

  Eigen::VectorXd GetDeviation(const size_t& index);

  sensor_msgs::JointState::ConstPtr GetGroundTruthJoints(const size_t& index);

  sensor_msgs::JointState::ConstPtr GetNoisyJoints(const size_t& index);

  void Load();

  void Store();

private:

  const std::string ground_truth_joints_topic_;
  const std::string noisy_joints_topic_;
  const std::string deviation_filename_;
};

#endif
