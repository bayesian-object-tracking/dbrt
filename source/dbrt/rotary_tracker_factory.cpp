/*
 * This is part of the Bayesian Robot Tracking
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
 * \file rotaty_tracker_factoy.cpp
 * \date June 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbrt/rotary_tracker_factory.h>

#include <ros/ros.h>
#include <dbot_ros/util/ros_interface.hpp>
#include <dbrt/builder/rotary_tracker_builder.hpp>

namespace dbrt
{
/**
 * \brief Create a gaussian filter tracking the robot joints based on joint
 *     measurements
 * \param prefix
 *     parameter prefix, e.g. fusion_tracker
 * \param kinematics
 *     URDF robot kinematics
 */
std::shared_ptr<dbrt::RotaryTracker> create_rotary_tracker(
    const std::string& prefix,
    const int& joint_count,
    const std::vector<int>& joint_order)
{
    ros::NodeHandle nh("~");

    typedef dbrt::RotaryTracker Tracker;

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    dbrt::FactorizedTransitionBuilder<Tracker>::Parameters
        transition_parameters;

    // linear state transition parameters
    transition_parameters.joint_sigmas = ri::read<std::vector<double>>(
        prefix + "joint_transition/joint_sigmas", nh);
    transition_parameters.bias_sigmas = ri::read<std::vector<double>>(
        prefix + "joint_transition/bias_sigmas", nh);
    transition_parameters.bias_factors = ri::read<std::vector<double>>(
        prefix + "joint_transition/bias_factors", nh);
    transition_parameters.joint_count = joint_count;

    auto transition_builder =
        std::make_shared<dbrt::FactorizedTransitionBuilder<Tracker>>(
            (transition_parameters));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbrt::RotarySensorBuilder<Tracker>::Parameters sensor_parameters;

    sensor_parameters.joint_sigmas = ri::read<std::vector<double>>(
        prefix + "joint_observation/joint_sigmas", nh);
    sensor_parameters.joint_count = joint_count;

    auto rotary_sensor_builder =
        std::make_shared<dbrt::RotarySensorBuilder<Tracker>>(sensor_parameters);

    /* ------------------------------ */
    /* - Build the tracker          - */
    /* ------------------------------ */
    auto tracker_builder = dbrt::RotaryTrackerBuilder<Tracker>(
        joint_count, joint_order, transition_builder, rotary_sensor_builder);

    return tracker_builder.build();
}
}

