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
 * \file ros_rbc_particle_filter_tracker_factory.hpp
 * \date March 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbrt/tracker/visual_tracker_factory.h>

#include <dbot_ros/util/ros_interface.hpp>
#include <dbot/builder/rb_sensor_builder.h>

#include <dbrt/builder/transition_builder.hpp>
#include <dbrt/builder/visual_tracker_builder.hpp>

namespace dbrt
{
/**
 * \brief Create a particle filter tracking the robot joints based on depth
 *     images measurements
 * \param prefix
 *     parameter prefix, e.g. fusion_tracker
 * \param urdf_kinematics
 *     URDF robot kinematics
 */
std::shared_ptr<dbrt::VisualTracker> create_visual_tracker(
    const std::string& prefix,
    const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
    const std::shared_ptr<dbot::ObjectModel>& object_model,
    const std::shared_ptr<dbot::CameraData>& camera_data)
{
    ros::NodeHandle nh("~");

    typedef dbrt::VisualTracker Tracker;
    typedef Tracker::State State;

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    dbrt::TransitionBuilder<Tracker>::Parameters transition_parameters;

    // linear state transition parameters
    transition_parameters.joint_sigmas = ri::read<std::vector<double>>(
        prefix + "joint_transition/joint_sigmas", nh);
    transition_parameters.joint_count = urdf_kinematics->num_joints();

    auto transition_builder =
        std::make_shared<dbrt::TransitionBuilder<Tracker>>(
            transition_parameters);

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

    auto sensor_builder =
        std::make_shared<dbot::RbSensorBuilder<State>>(
            object_model, camera_data, sensor_parameters);

    /* ------------------------------ */
    /* - Create Filter & Tracker    - */
    /* ------------------------------ */
    dbrt::VisualTrackerBuilder<Tracker>::Parameters tracker_parameters;
    tracker_parameters.evaluation_count = sensor_parameters.sample_count;

    tracker_parameters.moving_average_update_rate =
        ri::read<double>(prefix + "moving_average_update_rate", nh);
    tracker_parameters.max_kl_divergence =
        ri::read<double>(prefix + "max_kl_divergence", nh);
    tracker_parameters.sampling_blocks =
        ri::read<std::vector<std::vector<int>>>(prefix + "sampling_blocks", nh);

    auto tracker_builder =
        dbrt::VisualTrackerBuilder<Tracker>(urdf_kinematics,
                                            transition_builder,
                                            sensor_builder,
                                            object_model,
                                            camera_data,
                                            tracker_parameters);

    return tracker_builder.build();
}
}
