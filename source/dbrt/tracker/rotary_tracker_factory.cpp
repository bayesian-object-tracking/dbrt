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

#include <dbot_ros/util/ros_interface.h>
#include <dbrt/builder/rotary_tracker_builder.h>
#include <dbrt/tracker/rotary_tracker_factory.h>
#include <dbrt/util/parameter_tools.h>
#include <ros/ros.h>

namespace dbrt
{
/**
 * \brief Create a gaussian filter tracking the robot joints based on joint
 *     measurements
 * \param prefix
 *     parameter prefix, e.g. fusion_tracker
 * \param kinematics
 *     URDF robot kinematics
 * \param camera_frame_id
 *     This is the camera frame id or name used when the 
 *     estimate_camera_offset is on.
 */
std::shared_ptr<dbrt::RotaryTracker> create_rotary_tracker(
    std::string prefix,
    const std::shared_ptr<KinematicsFromURDF>& kinematics,
    sensor_msgs::JointState::ConstPtr joint_state)
{
    ros::NodeHandle nh("~");

    typedef dbrt::RotaryTracker Tracker;

    int joint_count = kinematics->num_joints();

    bool estimate_camera_offset =
        ri::read<bool>("camera_offset/estimate_camera_offset", nh);

    auto camera_observation_joint_sigmas_map = read_maps_from_map_list(
        "camera_offset/joint_observation/joint_sigmas", nh);
    auto camera_transition_joint_sigmas_map = read_maps_from_map_list(
        "camera_offset/joint_transition/joint_sigmas", nh);
    auto camera_joint_bias_sigmas_map = read_maps_from_map_list(
        "camera_offset/joint_transition/bias_sigmas", nh);
    auto camera_joint_bias_factors_map = read_maps_from_map_list(
        "camera_offset/joint_transition/bias_factors", nh);

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    dbrt::FactorizedTransitionBuilder<Tracker>::Parameters
        transition_parameters;

    auto observation_joint_sigmas_map =
        read_maps_from_map_list(prefix + "joint_observation/joint_sigmas", nh);
    auto transition_joint_sigmas_map =
        read_maps_from_map_list(prefix + "joint_transition/joint_sigmas", nh);
    auto joint_bias_sigmas_map =
        read_maps_from_map_list(prefix + "joint_transition/bias_sigmas", nh);
    auto joint_bias_factors_map =
        read_maps_from_map_list(prefix + "joint_transition/bias_factors", nh);

    if (estimate_camera_offset)
    {
        insert_map_with_prefixed_keys(
            camera_transition_joint_sigmas_map,
            kinematics->camera_frame_id() + "_",
            transition_joint_sigmas_map);


        insert_map_with_prefixed_keys(
            camera_joint_bias_sigmas_map,
            kinematics->camera_frame_id() + "_",
            joint_bias_sigmas_map);


        insert_map_with_prefixed_keys(
            camera_joint_bias_factors_map,
            kinematics->camera_frame_id() + "_",
            joint_bias_factors_map);
    }

    // linear state transition parameters
    transition_parameters.joint_sigmas =
        extract_ordered_values(transition_joint_sigmas_map, kinematics);
    transition_parameters.bias_sigmas =
        extract_ordered_values(joint_bias_sigmas_map, kinematics);
    transition_parameters.bias_factors =
        extract_ordered_values(joint_bias_factors_map, kinematics);
    transition_parameters.joint_count = joint_count;

    auto transition_builder =
        std::make_shared<dbrt::FactorizedTransitionBuilder<Tracker>>(
            (transition_parameters));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbrt::RotarySensorBuilder<Tracker>::Parameters sensor_parameters;

    if (estimate_camera_offset)
    {
        insert_map_with_prefixed_keys(
            camera_observation_joint_sigmas_map,
            kinematics->camera_frame_id() + "_",
            observation_joint_sigmas_map);
    }

    sensor_parameters.joint_sigmas =
        extract_ordered_values(observation_joint_sigmas_map, kinematics);

    sensor_parameters.joint_count = joint_count;

    auto rotary_sensor_builder =
        std::make_shared<dbrt::RotarySensorBuilder<Tracker>>(sensor_parameters);

    /* ------------------------------ */
    /* - Build the tracker          - */
    /* ------------------------------ */
    auto tracker_builder = dbrt::RotaryTrackerBuilder<Tracker>(
        kinematics, transition_builder, rotary_sensor_builder);

    auto tracker = tracker_builder.build();

    /* ------------------------------ */
    /* - Initialize tracker         - */
    /* ------------------------------ */
    std::vector<Eigen::VectorXd> initial_states_vectors = {
        kinematics->sensor_msg_to_eigen(*joint_state)};
    std::vector<dbrt::RobotState<>> initial_states;
    for (auto state : initial_states_vectors) initial_states.push_back(state);
    tracker->initialize(initial_states);

    return tracker;
}
}
