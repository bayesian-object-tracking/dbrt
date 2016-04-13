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
    const std::shared_ptr<robot_state_pub::RobotStatePublisher>& state_publisher)
    : transforms_provider_(state_publisher), tf_transformer_(false), is_empty_(true) {}


void RobotTransformer::set_joints(
    std::map<std::string, double> joints,
    const std::string& tf_prefix)
{
    // Store the joints values
    joints_ = joints;

    // These guys will be needed later to get transforms and query our transformer
    tf_prefix_ = tf_prefix;
    fake_time_ = ros::Time.now();

    // Lazy init if the transformer (only when actually queried)
    tf_transformer_.clear();
    is_empty_ = true;
}


void RobotTransformer::set_transformer_() const
{
    // Use method from robot_state_pub::RobotStatePublisher to get tfs
    std::vector<tf::StampedTransform> tf_transforms;
    transforms_provider_.getTransforms(joints_, fake_time_, tf_prefix_, tf_transforms);

    // Fill in our transformer with these transforms
    tf_transformer_.clear()
    for (tf_trans : tf_transforms)
    {
        ok = tf_transformer_.setTransform(tf_trans, authority="robot_transformer");
        if (!ok)
        {
            ROS_DEBUG("Problem setting tf from %s to %s", tf_trans.frame_id.c_str(),
                tf_trans.child_frame_id.c_str());
        }
    }
    is_empty = false;
}


void RobotTransformer::lookup_transform(const std::string& from,
    const std::string& to, tf::StampedTransform& tf_transform) const
{
    if (is_empty_)
    {
        set_transformer_();
    }
    tf_transformer_.lookupTransform(tf::resolve(tf_prefix_, from),
        tf::resolve(tf_prefix_, to), fake_time_, tf_transform);
}

} // end of namespace
