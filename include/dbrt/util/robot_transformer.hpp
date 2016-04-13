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
 * \author Cristina garcia Cifuentes (c.garciacifuentes@gmail.com)
 */

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <robot_state_pub/robot_state_publisher.h>
#include <dbrt/util/kinematics_from_urdf.hpp>


namespace dbrt
{

/**
 * \brief Lookup transforms between frame_ids given joint values.
 */
class RobotTransformer
{
public:
    /**
     * \brief Creates a RobotTransformer. Needs access to a publisher from robot_state_pub.
     */
    RobotTransformer(
        const std::shared_ptr<robot_state_pub::RobotStatePublisher>& state_publisher);

    /**
     * \brief Set joint configuration for which we want to calculate transforms.
     */
    void set_joints(std::map<std::string, double> joints, const std::string& tf_prefix);

    /**
     * \brief Lookup transform by frame_id pair.
     */
    void lookup_transform(const std::string& from, const std::string& to,
        tf::StampedTransform& tf_transform) const;

protected:
    void set_transformer_() const;

    // Joint values and corresponding tf_prefix.
    std::map<std::string, double> joints_;
    std::string tf_prefix_;

    // This guy knows about the kinematics and can provide tf_transforms from joints.
    // It happens to need a time to stamp the returned transforms.
    std::shared_ptr<robot_state_pub::RobotStatePublisher> transforms_provider_;
    std::Time fake_time_;

    // This guy knows how to build a tree of tf_transforms and do magical lookups.
    mutable tf::Transformer tf_transformer_;
    mutable bool is_empty_;
};

} // end of namespace
