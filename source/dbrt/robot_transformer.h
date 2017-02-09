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
 * \file robot_transformer.h
 * \date April 2016
 * \author C. Garcia Cifuentes (c.garciacifuentes@gmail.com)
 */

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <robot_state_pub/robot_state_publisher.h>
#include <dbrt/kinematics_from_urdf.h>


namespace dbrt
{

/**
 * \brief Lookup transforms between frame_ids given joint values.
 */
class RobotTransformer
{
public:
    /**
     * \brief Creates a RobotTransformer. Needs access to a robot publisher.
     */
    RobotTransformer(
        const std::shared_ptr<robot_state_pub::RobotStatePublisher>& state_publisher);

    /**
     * \brief Set joint configuration for which we want to calculate transforms.
     */
    void set_joints(const std::map<std::string, double>& joints);

    /**
     * \brief Acess to the transforms that were computed as by-product.
     */
    const std::vector<tf::StampedTransform>& get_transforms() const;

    /**
     * \brief Lookup transform between pair of frame names.
     */
    void lookup_transform(const std::string& from, const std::string& to,
        tf::StampedTransform& tf_transform) const;

protected:
    void set_lookup_provider_() const;

    // Joint values
    std::map<std::string, double> joints_;

    // This guy knows about the kinematics and can provide tf_transforms from joints.
    // It happens to need a time to stamp the returned transforms.
    std::shared_ptr<robot_state_pub::RobotStatePublisher> transforms_provider_;
    ros::Time fake_time_;
    std::string fake_prefix_;

    // This guy knows how to build a tree of tf_transforms and do magical lookups.
    mutable tf::Transformer lookup_provider_;
    mutable bool is_empty_;
    mutable std::vector<tf::StampedTransform> transforms_;
};

} // end of namespace
