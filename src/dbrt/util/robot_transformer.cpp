/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2016 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file robot_transformer.hpp
 * \date April 2016
 * \author Cristina Garcia Cifuentes (c.garciacifuentes@gmail.com)
 */

#pragma once

#include <ros/ros.h>

#include <dbrt/util/robot_transformer.hpp>


namespace dbrt
{

RobotTransformer::RobotTransformer(
  const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics)
  : robot_state_pub::RobotStatePublisher(urdf_kinematics->GetTree()),
  transformer_(false) {}


void RobotTransformer::set_transformer(
  std::map<std::string, double> joints,
  const std::string& tf_prefix)
{
  // These will be needed later to query our transformer
  tf_prefix_ = tf_prefix;
  fake_time_ = ros::Time.now();

  // Use method inherited from robot_state_pub::RobotStatePublisher
  std::vector<tf::StampedTransform> tf_transforms;
  getTransforms(joints, fake_time_, tf_prefix_, tf_transforms);

  // Fill in our transformer with these transforms
  transformer_.clear()
  for (tf_trans : tf_transforms)
  {
    ok = transformer_.setTransform(tf_trans, authority="robot_transformer");
    if (!ok)
    {
      ROS_DEBUG("Problem setting tf from %s to %s", tf_trans.frame_id.c_str(),
        tf_trans.child_frame_id.c_str());
    }
  }

}

void RobotTransformer::lookup_transform(const std::string& from,
  const std::string& to, tf::StampedTransform& transform) const
{
  transformer_.lookupTransform(tf::resolve(tf_prefix_, from),
    tf::resolve(tf_prefix_, to), fake_time_, transform);
}

} // end of namespace
