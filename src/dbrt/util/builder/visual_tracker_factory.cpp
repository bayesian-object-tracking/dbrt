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

/**
 * \file ros_rbc_particle_filter_tracker_factory.hpp
 * \date March 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbrt/util/builder/visual_tracker_factory.h>

#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot/tracker/builder/rb_observation_model_builder.h>

#include <dbrt/util/builder/transition_builder.hpp>
#include <dbrt/util/builder/visual_tracker_builder.hpp>

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
std::shared_ptr<dbrt::VisualTracker>
create_visual_tracker(
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
    dbrt::TransitionBuilder<Tracker>::Parameters params_state;

    // linear state transition parameters
    nh.getParam(prefix + "joint_transition/joint_sigmas",
                params_state.joint_sigmas);
    params_state.joint_count = urdf_kinematics->num_joints();

    auto state_trans_builder =
        std::make_shared<dbrt::TransitionBuilder<Tracker>>(
            params_state);

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbot::RbObservationModelBuilder<State>::Parameters params_obsrv;
    nh.getParam(prefix + "use_gpu", params_obsrv.use_gpu);

    if (params_obsrv.use_gpu)
    {
        nh.getParam(prefix + "gpu/sample_count", params_obsrv.sample_count);
    }
    else
    {
        nh.getParam(prefix + "cpu/sample_count", params_obsrv.sample_count);
    }

    nh.getParam(prefix + "observation/occlusion/p_occluded_visible",
                params_obsrv.occlusion.p_occluded_visible);
    nh.getParam(prefix + "observation/occlusion/p_occluded_occluded",
                params_obsrv.occlusion.p_occluded_occluded);
    nh.getParam(prefix + "observation/occlusion/initial_occlusion_prob",
                params_obsrv.occlusion.initial_occlusion_prob);

    nh.getParam(prefix + "observation/kinect/tail_weight",
                params_obsrv.kinect.tail_weight);
    nh.getParam(prefix + "observation/kinect/model_sigma",
                params_obsrv.kinect.model_sigma);
    nh.getParam(prefix + "observation/kinect/sigma_factor",
                params_obsrv.kinect.sigma_factor);
    params_obsrv.delta_time = 1. / 6.;

    // gpu only parameters
    nh.getParam(prefix + "gpu/use_custom_shaders",
                params_obsrv.use_custom_shaders);
    nh.getParam(prefix + "gpu/vertex_shader_file",
                params_obsrv.vertex_shader_file);
    nh.getParam(prefix + "gpu/fragment_shader_file",
                params_obsrv.fragment_shader_file);
    nh.getParam(prefix + "gpu/geometry_shader_file",
                params_obsrv.geometry_shader_file);

    auto obsrv_model_builder =
        std::make_shared<dbot::RbObservationModelBuilder<State>>(
            object_model, camera_data, params_obsrv);

    /* ------------------------------ */
    /* - Create Filter & Tracker    - */
    /* ------------------------------ */
    dbrt::VisualTrackerBuilder<Tracker>::Parameters
        params_tracker;
    params_tracker.evaluation_count = params_obsrv.sample_count;
    nh.getParam(prefix + "moving_average_update_rate",
                params_tracker.moving_average_update_rate);
    nh.getParam(prefix + "max_kl_divergence", params_tracker.max_kl_divergence);
    ri::ReadParameter(
        prefix + "sampling_blocks", params_tracker.sampling_blocks, nh);

    auto tracker_builder =
        dbrt::VisualTrackerBuilder<Tracker>(urdf_kinematics,
                                                            state_trans_builder,
                                                            obsrv_model_builder,
                                                            object_model,
                                                            camera_data,
                                                            params_tracker);

    return tracker_builder.build();
}
}
