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

/*
 * This file implements a part of the algorithm published in:
 *
 * M. Wuthrich, J. Bohg, D. Kappler, C. Pfreundt, S. Schaal
 * The Coordinate Particle Filter -
 * A novel Particle Filter for High Dimensional Systems
 * IEEE Intl Conf on Robotics and Automation, 2015
 * http://arxiv.org/abs/1505.00251
 *
 */

/**
 * \file ros_particle_tracker_factory.hpp
 * \date March 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbot/builder/rb_sensor_builder.h>
#include <dbot_ros/util/ros_interface.h>
#include <dbrt/builder/visual_tracker_builder.h>
#include <dbrt/tracker/visual_tracker_factory.h>
#include <dbrt/urdf_object_loader.h>
#include <dbrt/util/parameter_tools.h>

namespace dbrt
{
/**
 * \brief Create a particle filter tracking the robot joints based on depth
 *     images measurements
 * \param prefix
 *     parameter prefix, e.g. fusion_tracker
 * \param kinematics
 *     URDF robot kinematics
 */
std::shared_ptr<dbrt::VisualTracker> create_visual_tracker(
    std::string prefix,
    std::shared_ptr<KinematicsFromURDF> kinematics,
    std::shared_ptr<dbot::CameraData> camera_data,
    sensor_msgs::JointState::ConstPtr joint_state)
{
    ros::NodeHandle nh("~");

    typedef dbrt::VisualTracker Tracker;
    typedef Tracker::State State;

    bool estimate_camera_offset =
        ri::read<bool>("camera_offset/estimate_camera_offset", nh);

    auto camera_transition_joint_sigmas_map = read_maps_from_map_list(
        "camera_offset/joint_transition/joint_sigmas", nh);

    /* ------------------------------ */
    /* - Create the robot model     - */
    /* ------------------------------ */
    auto object_model_loader =
        std::make_shared<dbrt::UrdfObjectModelLoader>(kinematics);

    // Load the model usign the URDF loader
    auto object_model = std::make_shared<dbot::ObjectModel>(
        std::make_shared<dbrt::UrdfObjectModelLoader>(kinematics), false);

    ROS_INFO("Robot model loaded");

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    dbrt::TransitionBuilder<Tracker>::Parameters transition_parameters;

    auto transition_joint_sigmas_map =
        read_maps_from_map_list(prefix + "joint_transition/joint_sigmas", nh);
    if (estimate_camera_offset)
    {
       insert_map_with_prefixed_keys(camera_transition_joint_sigmas_map, 
                                     kinematics->camera_frame_id() + "_", 
                                     transition_joint_sigmas_map);
    }

    // linear state transition parameters
    transition_parameters.joint_sigmas =
        extract_ordered_values(transition_joint_sigmas_map, kinematics);
    ROS_INFO("Transition parameter loaded");
    transition_parameters.joint_count = kinematics->num_joints();
    PV(transition_parameters.joint_count);

    ROS_INFO("Transition parameter loaded");

    auto transition_builder =
        std::make_shared<dbrt::TransitionBuilder<Tracker>>(
            transition_parameters);

    ROS_INFO("Transition model created");

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbot::RbSensorBuilder<State>::Parameters sensor_parameters;

    sensor_parameters.use_gpu = ri::read<bool>(prefix + "use_gpu", nh);

    if (sensor_parameters.use_gpu)
    {
        sensor_parameters.sample_count =
            ri::read<int>(prefix + "gpu/sample_count", nh);
    }
    else
    {
        sensor_parameters.sample_count =
            ri::read<int>(prefix + "cpu/sample_count", nh);
    }

    sensor_parameters.occlusion.p_occluded_visible = ri::read<double>(
        prefix + "observation/occlusion/p_occluded_visible", nh);
    sensor_parameters.occlusion.p_occluded_occluded = ri::read<double>(
        prefix + "observation/occlusion/p_occluded_occluded", nh);
    sensor_parameters.occlusion.initial_occlusion_prob = ri::read<double>(
        prefix + "observation/occlusion/initial_occlusion_prob", nh);

    sensor_parameters.kinect.tail_weight =
        ri::read<double>(prefix + "observation/kinect/tail_weight", nh);
    sensor_parameters.kinect.model_sigma =
        ri::read<double>(prefix + "observation/kinect/model_sigma", nh);
    sensor_parameters.kinect.sigma_factor =
        ri::read<double>(prefix + "observation/kinect/sigma_factor", nh);
    sensor_parameters.delta_time =
        ri::read<double>(prefix + "observation/delta_time", nh);

    // gpu only parameters
    sensor_parameters.use_custom_shaders =
        ri::read<bool>(prefix + "gpu/use_custom_shaders", nh);
    sensor_parameters.vertex_shader_file =
        ri::read<std::string>(prefix + "gpu/vertex_shader_file", nh);
    sensor_parameters.fragment_shader_file =
        ri::read<std::string>(prefix + "gpu/fragment_shader_file", nh);
    sensor_parameters.geometry_shader_file =
        ri::read<std::string>(prefix + "gpu/geometry_shader_file", nh);

    auto sensor_builder = std::make_shared<dbot::RbSensorBuilder<State>>(
        object_model, camera_data, sensor_parameters);

    ROS_INFO("Observation model created");

    /* ------------------------------ */
    /* - Create Filter & Tracker    - */
    /* ------------------------------ */
    dbrt::VisualTrackerBuilder<Tracker>::Parameters tracker_parameters;
    tracker_parameters.evaluation_count = sensor_parameters.sample_count;

    tracker_parameters.moving_average_update_rate =
        ri::read<double>(prefix + "moving_average_update_rate", nh);
    tracker_parameters.max_kl_divergence =
        ri::read<double>(prefix + "max_kl_divergence", nh);

    auto sampling_blocks_definition =
        ri::read<SamplingBlocksDefinition>("sampling_blocks", nh);

    auto camera_offset_sampling_blocks_definition =
        ri::read<SamplingBlocksDefinition>("camera_offset/sampling_blocks", nh);

    if (estimate_camera_offset)
    {
        sampling_blocks_definition = merge_sampling_block_definitions(
	   sampling_blocks_definition,
           camera_offset_sampling_blocks_definition, kinematics->camera_frame_id() + '_');
    }

    tracker_parameters.sampling_blocks =
        definition_to_sampling_block(sampling_blocks_definition, kinematics);

    auto tracker_builder =
        dbrt::VisualTrackerBuilder<Tracker>(kinematics,
                                            transition_builder,
                                            sensor_builder,
                                            object_model,
                                            camera_data,
                                            tracker_parameters);

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
