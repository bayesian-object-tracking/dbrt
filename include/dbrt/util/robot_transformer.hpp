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
 * \brief Lookup transforms between frame_ids given joint values
 */
class RobotTransformer : robot_state_pub::RobotStatePublisher
{
public:
    RobotTransformer(const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics);

    void set_transforner(std::map<std::string, double> joints, const std::string& tf_prefix);

    void lookup_transform(const std::string& from, const std::string& to,
      tf::StampedTransform& transform) const;

protected:
    std::string tf_prefix_;
    std::Time fake_time_;
    tf::Transformer transformer_;

};

} // end of namespace
